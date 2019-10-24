(define-public (append-space txt) (string-append txt " "))
(display "======================")
(define-public (ghost-str-response)
    (let ((gr (ghost-get-result)))
         (if (eq? gr '())
             ""
             (string-drop-right
                (string-concatenate
                    (map append-space (map cog-name gr)))
                1)
         )
    )
)
