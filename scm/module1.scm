(use-modules (opencog)
             (opencog exec)
             (opencog query)
             (opencog openpsi)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog ghost)
             (opencog ghost procedures))

(ghost-set-sti-weight 0)
(ghost-af-only #f)


(define-public (append-space txt) (string-append txt " "))
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
