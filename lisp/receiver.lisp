; gb_receiver as a VESC Express Lisp script.
;
; Serves the same 0xABF0 BLE service the receiver firmware does, so an
; UNMODIFIED GS-THUMB remote connects to it: same device name, same 3-byte
; throttle write, same 65-byte telemetry frame.
;
; VESC Tool > VESC Express > Bluetooth:
;   Mode = "Enabled with Scripting"
;   BLE Service Capacity >= 1
;   BLE Characteristic and Descriptor Capacity >= 6
; This takes over the radio: VESC Tool can no longer connect over Bluetooth.
;
; The receiver board must be off the CAN bus — two throttle sources on one bus
; fight each other.

; ---------------- board config ----------------
; The remote does its own speed math from these, so they must match the VESC.
(def motor-poles 14)      ; Motor Settings > "Motor Poles"
(def gear-ratio 1.0)      ; motor turns per wheel turn (VESC "Gear Ratio")
(def wheel-diam 0.09)     ; meters (VESC "Wheel Diameter" / 1000)
(def aux-pin -1)          ; GPIO driving the aux output, -1 = not fitted

(def brake-max 0.3)       ; failsafe brake, fraction of each VESC's l_current_min
(def brake-ramp 10.0)     ; seconds to reach brake-max
(def link-timeout 0.2)    ; seconds without a throttle packet = link lost
(def dropout-timeout 0.5) ; seconds without a CAN status msg = VESC dropout
(def stopped-erpm 300)    ; below this the vehicle counts as stopped

(def neutral 128)

; ---------------- state ----------------
(def throttle neutral)
(def aux-state 0)
(def last-rx (systime))
(def rx-seen false)       ; no failsafe before the first packet ever arrives
(def fs-active false)
(def fs-start (systime))
(def devs nil)

; An eeprom address that has never been written reads back nil.
(defun eeprom-or (v d) (if (eq v nil) d v))

; addr 0 = trip km
(def trip-km (let ((v (eeprom-or (eeprom-read-f 0) 0.0)))
               (if (and (>= v 0.0) (< v 100000.0)) v 0.0)))
(def trip-saved trip-km)

; ---------------- CAN helpers ----------------
(defun drive (rel) (loopforeach id devs (canset-current-rel id rel)))
(defun brake (rel) (loopforeach id devs (canset-brake-rel id rel)))

; Signed ERPM of the fastest-spinning VESC — the direction reference for
; brake-vs-reverse, same as bldc_interface_can_get_signed_dominant_erpm().
(defun dom-erpm ()
    (let ((best 0.0))
        (progn
            (loopforeach id devs
                (let ((r (canget-rpm id)))
                    (if (> (abs r) (abs best)) (setq best r) nil)))
            best)))

(defun link-lost ()
    (let ((lost (eq devs nil)))
        (progn
            (loopforeach id devs
                (let ((age (can-msg-age id 1)))
                    (if (or (eq age nil) (> age dropout-timeout))
                        (setq lost true) nil)))
            lost)))

(defun speed-kmh (erpm)
    (let ((wheel-rpm (/ (/ (abs erpm) (/ motor-poles 2.0)) gear-ratio)))
        (/ (* wheel-rpm 3.14159 wheel-diam 60.0) 1000.0)))

(defun set-aux (s)
    (let ((v (if (= s 0) 0 1)))
        (if (not (= v aux-state))
            (progn
                (setq aux-state v)
                (eeprom-store-i 1 v)
                (if (>= aux-pin 0) (gpio-write aux-pin v) nil))
            nil)))

; ---------------- BLE ----------------
; UUIDs are written in text order, so 0xABF0 is [0xab 0xf0].
(ble-set-name "GS-THUMB")
(ble-start-app)

; Services outlive a script restart, so re-uploading hits "Too many services"
; unless the old one is cleared first. Services must be removed last-created
; first; we only ever create one.
(loopforeach s (ble-get-services) (ble-remove-service s))

(def handles (ble-add-service [0xab 0xf0] '(
    ((uuid . [0xab 0xf1])                       ; throttle in
     (prop . (prop-read prop-write prop-write-nr))
     (max-len . 20))
    ((uuid . [0xab 0xf2])                       ; telemetry out
     (prop . (prop-read prop-notify))
     (max-len . 65)
     (descr . (((uuid . [0x29 0x02]) (max-len . 2) (default-value . [0 0])))))
    ((uuid . [0xab 0xf3])                       ; command in
     (prop . (prop-read prop-write prop-write-nr))
     (max-len . 20))
    ((uuid . [0xab 0xf4])                       ; status/ack out
     (prop . (prop-read prop-notify))
     (max-len . 20)
     (descr . (((uuid . [0x29 0x02]) (max-len . 2) (default-value . [0 0])))))
)))

; Returned in definition order, service handle first; char declaration handles
; are not included.
(def h-throttle (ix handles 1))
(def h-telem    (ix handles 2))
(def h-cmd      (ix handles 4))
(def h-status   (ix handles 5))

(defun on-ble-rx (handle data)
    (cond
        ((and (= handle h-throttle) (>= (buflen data) 3))
            (progn
                ; clamp at the trust boundary: downstream treats this as a byte
                (let ((v (bufget-u16 data 0)))
                    (setq throttle (if (> v 255) 255 v)))
                (setq last-rx (systime))
                (setq rx-seen true)
                (set-aux (bufget-u8 data 2))))
        ((and (= handle h-cmd) (>= (buflen data) 1) (= (bufget-u8 data 0) 1))
            (progn
                (setq trip-km 0.0)
                (setq trip-saved 0.0)
                (eeprom-store-f 0 0.0)
                (ble-attr-set-value h-status [1 0])))
        (t nil)))

(defun ble-handler ()
    (loopwhile t
        (recv ((event-ble-rx (? handle) (? data)) (on-ble-rx handle data))
              (_ nil))))

; ---------------- control ----------------
; Smart reverse, ported from throttle.c: past neutral against the direction of
; travel is a brake, with the direction of travel is drive. Everything scales
; against each VESC's own current limits.
(defun throttle-step ()
    (if (or (and rx-seen (> (secs-since last-rx) link-timeout)) (link-lost))
        (progn (setq fs-active true) (setq fs-start (systime)) (drive 0.0))
        (let ((v throttle) (erpm (dom-erpm)))
            (cond
                ((= v neutral) (drive 0.0))
                ((> v neutral)
                    (let ((m (/ (- v neutral) 127.0)))
                        (if (< erpm (- 0 stopped-erpm)) (brake m) (drive m))))
                (t
                    (let ((m (/ (- neutral v) 128.0)))
                        (if (> erpm stopped-erpm) (brake m) (drive (- 0.0 m)))))))))

; Ramped regen brake, ported from failsafe.c. Brake current opposes actual
; rotation, so this is direction-safe and releases itself once stopped.
(defun failsafe-step ()
    (let ((moving (> (abs (dom-erpm)) stopped-erpm)))
        (progn
            (if moving
                (let ((p (/ (secs-since fs-start) brake-ramp)))
                    (brake (* brake-max (if (> p 1.0) 1.0 p))))
                (brake 0.0))
            ; Clear only with the link back AND stopped — never while moving.
            (if (and (not moving)
                     (< (secs-since last-rx) link-timeout)
                     (not (link-lost)))
                (progn (setq throttle neutral) (setq fs-active false))
                nil))))

(defun control-loop ()
    (loopwhile t
        (progn
            (if fs-active (failsafe-step) (throttle-step))
            (sleep 0.02))))                     ; 50 Hz

; ---------------- telemetry ----------------
; 65-byte frame, little-endian (bufset default), byte layout fixed by the
; remote's parser. The BMS block (bytes 14-54) is left at zero on purpose: the
; remote reads a zero BMS voltage as "no BMS fitted" and falls back to pack
; voltage against the cell count and chemistry configured on the remote. Fake a
; BMS voltage here and it takes the BMS branch instead, where zero capacity
; means the board battery gauge never updates.
(defun telem-loop ()
    (let ((buf (bufcreate 65)) (last (systime)))
        (loopwhile t
            (progn
                (setq devs (can-list-devs))
                (bufclear buf)
                (let ((id (if (eq devs nil) nil (ix devs 0)))
                      (erpm (dom-erpm))
                      (dt (secs-since last)))
                    (progn
                        (setq last (systime))
                        (if (eq id nil) nil
                            (progn
                                (bufset-i16 buf 0  (* (canget-temp-fet id) 100))
                                (bufset-i16 buf 2  (* (canget-temp-motor id) 100))
                                (bufset-i16 buf 4  (* (canget-current id) 100))
                                (bufset-i16 buf 6  (* (canget-current-in id) 100))
                                (bufset-i32 buf 8  erpm)
                                (bufset-i16 buf 12 (* (canget-vin id) 100))))
                        (setq trip-km (+ trip-km (* (speed-kmh erpm) (/ dt 3600.0))))
                        (if (> (- trip-km trip-saved) 0.1)   ; commit every 100 m
                            (progn (eeprom-store-f 0 trip-km) (setq trip-saved trip-km))
                            nil)))
                (bufset-u8  buf 55 motor-poles)
                (bufset-u16 buf 56 (* gear-ratio 1000))
                (bufset-u16 buf 58 (* wheel-diam 1000))
                (bufset-u8  buf 60 aux-state)
                (bufset-i32 buf 61 (* trip-km 100))
                (ble-attr-set-value h-telem buf)
                (sleep 0.1)))))                 ; 10 Hz

; ---------------- start ----------------
(if (>= aux-pin 0)
    (progn
        (gpio-configure aux-pin 'pin-mode-out)
        (setq aux-state (if (= (eeprom-or (eeprom-read-i 1) 0) 0) 0 1))
        (gpio-write aux-pin aux-state))
    nil)

(can-scan)                                      ; blocking, once
(setq devs (can-list-devs))
(drive 0.0)                                     ; known-quiescent before anything else

(event-register-handler (spawn ble-handler))
(event-enable 'event-ble-rx)
(spawn control-loop)
(spawn telem-loop)
