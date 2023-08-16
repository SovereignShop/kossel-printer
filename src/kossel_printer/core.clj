(ns kossel-printer.core
  (:require
   [plexus.utils :as pu]
   [plexus.transforms :as tf]
   [clojure.java.io :as io]
   [clj-manifold3d.core :as m]
   [kossel-printer.utils :as u]
   [kossel-printer.math :refer [pi pi|2 pi|3 pi|4 pi|5 pi|6 two-pi
                                TT T T|2 T|3 T|4 T|5 T|6
                                sin asin cos acos sqrt sqr atan tan]]
   [kossel-printer.offset-polygon :refer [offset-polygon]]
   [sicmutils.env :as e]
   [sicmutils.generic :refer [dot-product magnitude negate cross-product]]
   [plexus.core :as paths
    :refer [forward hull left right up down translate
            set segment transform branch offset insert
            rotate frame save-transform add-ns extrude to points loft
            result difference union export-models export
            intersection mirror lookup-transform add-ns get-model get-model]]))

(def rod-mount-offset 21)
(def rod-mount-margin (- 65/2 rod-mount-offset 3))

(def printer-height (u/in->mm 48))
(def heated-bed-radius 310/2)
(def build-plate-radius 310/2)
(def carriage-spacer-length 2.5)
(def outer-mount-length (- rod-mount-margin carriage-spacer-length))
(def heated-bed-mask-radius (+ 60 heated-bed-radius))
(def heated-bed-mask-radius-inches (u/mm->in heated-bed-mask-radius))

(def heated-bed-shield-width
  (/ (- (u/circumference (+ 1 heated-bed-mask-radius))
        (* 3 46))
     3))

(def heated-bed-shield-width-inches
  (u/mm->in heated-bed-shield-width))

(def printer-offset 400)

(def panel-width (u/in->mm 22))
(def panel-height (u/in->mm 48))
(def panel-thickness 4)

;; Offset from edge of vertical aluminum extrusion that faces the build plate.
;; Offset is towards the center of the plate.
(def stepper-mount-frame-offset 38)
(def belt-shaft-mask-offset 19)
(def belt-shaft-width 30)

(def ^:export-model printer-model
  (extrude
   (result :name :printer-model :expr :body)
   (frame :cross-section (m/square 20.4 20.4 true) :name :body)
   (for [x (range 1)]
     (branch
      :from :body
      (add-ns :namespace (keyword (str "k" x)))
      (branch
       :from :body
       :with []
       (frame :name :side-panel
              :cross-section (m/square panel-width panel-height true))

       (translate :z (/ panel-height 2))
       (rotate :x pi|2)
       (translate :x (+ (/ build-plate-radius 2) 40))
       (forward :length panel-thickness))
      (branch
       :from :body
       (translate :z 60)
       (frame :cross-section (m/circle heated-bed-radius)
              :name :body
              :fn 120)
       (forward :length 8)
       (save-transform :frame :body :name :bed)
       (branch
        :from :body :with []
        (frame :cross-section (m/circle heated-bed-mask-radius 300)
               :name :heated-bed-mask)
        (forward :length 200))
       (translate :x build-plate-radius)
       #_(branch
          :from :body
          :with []
          (segment extruder-assembly-body-fisheye-mechanical-base)))
      (for [i (range 3)]
        (branch
         :from :body
         (rotate :z (* i (/ (* 2 pi) 3)))
         (add-ns :namespace (case i 0 :e0 1 :e1 2 :e2))

         (branch
          :from :body
          (translate :x 235)
          (save-transform :frame :body :name :vertical-rod)

          (branch
           :from :body :with []
           (frame :name :belt-shaft-mask
                  :cross-section (m/square belt-shaft-width belt-shaft-width true))
           (translate :x (- (+ 10 belt-shaft-mask-offset)))
           (forward :length printer-height))

          (branch
           :from :body
           (rotate :y pi|2)
           (translate :z 10)
           (save-transform :frame :body :name :frame-support-mount))
          (branch
           :from :body :with []
           (frame :cross-section (-> #_(m/square 50 68 true)
                                     (let [cr 1.5]
                                       (m/cross-section
                                        (points
                                         :axes [:x :z]
                                         (frame :name :origin :curve-radius cr)
                                         (rotate :y (- pi|2))
                                         (forward :length 10)
                                         (right :angle pi|2)
                                         (left :angle pi|2)
                                         (forward :length (- 68/2 (* 3 cr) 10))
                                         (left :angle pi|2)
                                         (forward :length (- 54 (* 2 cr)))
                                         (left :angle pi|2)
                                         (forward :length (- 68 (* 2 cr)))
                                         (left :angle pi|2)
                                         (forward :length (- 54 (* 2 cr)))
                                         (left :angle pi|2)
                                         (forward :length (- 68/2 (* 3 cr) 10))
                                         (left :angle pi|2)
                                         (right :angle pi|2)
                                         (forward :length 10))))
                                     (m/rotate (- T|2))
                                     (m/translate [10.2 0])
                                     #_(m/offset (- 5) :square)
                                     #_(m/offset 5 :round)
                                     #_(m/simplify 0.1))
                  :name :carriage-mask)

           #_(translate :x 0)
           (translate :z 66)
           (forward :length (- printer-height 66 25)))
          (forward :length printer-height))

         (branch
          :from :body
          :with []
          (frame :name :side-rod-mask
                 :cross-section (m/square 20.4 400 true))

          (rotate :z pi|3)
          (translate :x 145)
          (forward :length 61.5)
          (set :cross-section (m/union
                               (for [sign [- +]
                                     y (take 4 (iterate #(- % 19) 190))]
                                 (-> (m/square 20.4 13 true)
                                     (m/translate [0 (sign y)])))))
          (forward :length 0.9))

         (for [[ns_ z] [[:r0 0]
                        [:r1 40]
                        [:r2 (- printer-height 20 220)]
                        [:r3 (- printer-height 20)]]]
           (branch
            :from :body
            (translate :z z)
            (add-ns :namespace ns_)
            (rotate :z (/ Math/PI 3))
            (translate :x (case ns_ (:r2 :r3) 160 145))
            (save-transform :frame :body :name :side-rod)
            (branch
             :from :body
             (rotate :y (- (/ Math/PI 2)))
             (translate :x 10 :z 10)
             (save-transform :frame :body :name :plate-support-mount-point))
            (branch
             :from :body
             :with []
             (frame :name :side-rod-bolt-point
                    :cross-section (m/square 7 11 true)
                    :fn 50)
             (for [i (range 2)]
               (branch
                :from :side-rod-bolt-point
                (translate :z 10)
                (rotate :x (* i pi))
                (translate :y (- (/ 360 2) 12))
                (rotate :y pi|2)
                (for [x [0 -25]]
                  (branch
                   :from :side-rod-bolt-point
                   (translate :y x)
                   (forward :length 13)
                   (set :cross-section (m/circle 14/2))
                   (forward :length 50))))))
            (translate :z 10)
            (rotate :x (/ Math/PI 2))
            (forward :length 400 :center true)))))))))

(lookup-transform printer-model :k0.e0.r2/side-rod)

(defn vec-sub [X Y]
  (mapv - X Y))

(def top-key-points
  (let [pts
        (for [i (range 2)
              tf [(-> (lookup-transform printer-model :k0.e1.r3/side-rod)
                      (m/translate [16 0 0]))
                  (-> (lookup-transform printer-model :k0.e2.r3/side-rod)
                      (m/translate [16 0 0]))]]
          (let [[x y] (-> tf
                          (m/translate [(if (odd? i) 120 0) (* i 415) 0])
                          (m/rotate [0 0 (* i T)])
                          (tf/translation-vector)
                          (subvec 0 2))]
            [(u/mm->in x) (u/mm->in y)]))
        origin-pt (first pts)]
    pts
    #_(for [pt pts]
        (vec-sub pt origin-pt))))

(m/union
 (for [pt top-key-points]
   (-> (m/square 10 10 true)
       (m/translate (mapv u/in->mm pt))
       (m/extrude 90)
       (m/translate [0 0 (u/in->mm 48)]))))

(def linear-print-frarm
  (m/union
   (-> (m/square panel-width panel-height)
       (m/extrude panel-thickness)
       (m/translate [-200 -250 (u/in->mm 48)]))
   (m/union
    (for [pt top-key-points]
      (-> (m/square 10 10 true)
          (m/translate (mapv u/in->mm pt))
          (m/extrude 90)
          (m/translate [0 0 (u/in->mm 48)]))))


   (m/union
    (for [i (range 2)]
      (-> (m/union (get-model printer-model :body)
                   (get-model printer-model :heated-bed-mask)
                   (-> (m/cube 10 10 90 true)
                       (m/transform (-> (lookup-transform printer-model :k0.e1.r3/side-rod)
                                        (m/translate [16 0 0]))))
                   (-> (m/cube 10 10 90 true)
                       (m/transform (-> (lookup-transform printer-model :k0.e2.r3/side-rod)
                                        (m/translate [16 0 0])))))
          (m/rotate [0 0 (* i T)])
          (m/translate [(if (odd? i) 120 0) (* i 415) 0]))))))

#_(def ^:export-model radial-print-farm
    (for [i (range 6)]
      (branch
       :from :body
       (rotate :z (* x pi|3))
       (translate :x -400))))

(def build-plate-mask
  (extrude
   (frame :name :build-plate-mask
          :cross-section (m/circle build-plate-radius 300))

   (transform :replace (lookup-transform printer-model :k0/bed))
   (forward :length 100 :center true)))

(def ^:export-mount build-plate-support
  (extrude
   (result :name :build-plate-support :expr
           (difference :build-plate-support-body :build-plate-mask
                       :build-plate-support-bolt-mask))

   (frame :cross-section (m/square 20 20 true)
          :name :build-plate-support-body)

   (transform :replace (lookup-transform printer-model :k0.e0.r1/side-rod))

   (for [sign [+]]
     (branch
      :from :build-plate-support-body
      (translate :y (sign 95) :x -20)
      #_(show-coordinate-frame :label "Build Plate Support")

      (branch
       :from :build-plate-support-body
       :with []
       (frame :cross-section (m/hull (-> (m/circle (+ 0.15 3/2) 40))
                                     (-> (m/circle (+ 0.15 3/2) 40)
                                         (m/translate [6 0])))
              :name :build-plate-support-bolt-mask)
       (translate :z 15)
       (rotate :y (- pi|2))
       (translate :z -10)
       (forward :length 3)
       (set :cross-section  (m/hull (-> (m/circle 5 40)
                                        (m/translate [-6 0]))
                                    (-> (m/circle 5 40)
                                        (m/translate [6 0]))) )
       #_(show-coordinate-frame)
       (forward :length 50))

      (forward :length 30)))

   (branch
    :from :build-plate-support-body
    :with []
    (:forms build-plate-mask))))

(def psu-width 51)
(def psu-length 115)
(def psu-height 215)

(def psu-x-distance-between-bolts 50)
(def psu-y-distance-between-bolts 150)

(def ^:export-model psu-mount
  (extrude
   (result :name :psu-mount
           :expr (difference :psu-mount-body :psu-bolt-mask :psu-mount-bolt-mask))

   (frame :cross-section (m/square 6 30 true)
          :name :psu-mount-body)

   (transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))
   (translate :z 700 :x (+ 3 10.2))

   (branch
    :from :psu-mount-body
    :with []
    (frame :name :psu-mount-bolt-mask
           :cross-section (m/circle 7))
    (rotate :y (- pi|2))
    (translate :z (- 3))
    (forward :length 3)
    (set :cross-section (m/square 11 7 true))
    (forward :length 4))

   (for [i (range 2)]
     (branch
      :from :psu-mount-body
      (rotate :x (+ (* i pi) pi|2))
      (forward :length 10)
      (right :angle pi|4 :curve-radius 3)
      (left :angle pi|4 :curve-radius 3)
      (forward :length 20)))

   (branch
    :from :psu-mount-body
    :with []
    (frame :cross-section
           (m/union
            (for [y [(- (/ psu-x-distance-between-bolts 2))
                     (/ psu-x-distance-between-bolts 2)]]
              (-> (m/circle (+ 0.15 3))
                  (m/translate [0 y]))))
           :name :psu-bolt-mask)
    (rotate :y pi|2)
    (translate :z (- 3))
    (forward :length 5)
    (offset :delta -1)
    (forward :length 5))))

(def ^:export-model extruder-mount
  (extrude
   (result :name :extruder-mount
           :expr (difference :extruder-mount-body :stepper-mask :bolt-mask))
   (frame :cross-section (m/square 4 82 true)
          :name :extruder-mount-body
          :fn 50)
   (transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))
   (translate :z 400 :x (+ 10.2 4/2) :y 10)

   (branch
    :from :extruder-mount-body
    (frame :name :bolt-mask :cross-section (m/circle 7))
    (for [z-off [-31 31]]
      (branch
       :from :extruder-mount-body
       :with [:bolt-mask]
       (set :cross-section (m/circle 7))
       (translate :x 2 :z z-off :y -10)
       (rotate :y (- pi|2))
       (forward :length 2)
       (set :cross-section (m/square 11 7 true))
       (forward :length 3))))

   (rotate :x pi|2)
   (forward :length 20)
   (right :angle pi|4 :curve-radius 4/2)
   (left :angle pi|4 :curve-radius 4/2)
   (forward :length 6)
   (right :angle pi|2 :curve-radius 4/2)
   (set :cross-section (m/square 9 82 true))
   (translate :x 2.5)
   (forward :length 25)
   (branch
    :from :extruder-mount-body
    :with []
    (frame :name :stepper-mask
           :cross-section
           (m/union (m/circle 12)
                    (m/union
                     (let [o (/ (u/in->mm 1.25) 2)]
                       (for [x [(- o) o]
                             y [(- o) o]]
                         (-> (m/circle 2)
                             (m/translate [x y])))))) )
    (rotate :y (- pi|2))
    (forward :length 20 :center true))
   (forward :length 25)))

(def ^:export-model board-box
  (extrude
   (result :name :board-mount
           :expr (difference (union :board-mount-body :bottom-wall)
                             :mount-bolt-mask))

   (frame :name :board-mount-body
          :cross-section (m/square 3 140 true)
          :curve-radius 3/2
          :fn 40)

   (transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))
   (translate :z 700 :x (+ 3/2 10.2))

   (save-transform :frame :board-mount-body :name ::start-frame)

   (let [[left-pts right-pts]
         (for [i (range 2)]
           (points
            :axes [:x :y]
            (frame :name :bottom-wall :curve-radius 3/2)
            (rotate :x (+ (* i pi) pi|2))
            (forward :length 10)
            (right :angle pi|4)
            (left :angle pi|4)
            (forward :length 45)
            (right :angle pi|2)
            (forward :length 30)))
         all-pts (concat (rseq left-pts) right-pts)]
     (branch
      :from :board-mount-body
      :with []
      (frame :name :bottom-wall
             :cross-section (m/cross-section (offset-polygon all-pts -1.0)))
      (translate :z (- 140/2))
      (forward :length 3)))

   (branch
    :from :board-mount-body
    :with []
    (frame :cross-section (m/circle 7) :name :mount-bolt-mask)
    (for [z-off [-50 50]]
      (branch
       :from :mount-bolt-mask
       (translate :x (+ 0.1 3/2) :z z-off)
       (rotate :y (- pi|2))
       (forward :length 2)
       (set :cross-section (m/square 11 7 true))
       (forward :length 2)
       #_(show-coordinate-frame))))

   (for [i (range 2)]
     (branch
      :from :board-mount-body
      (rotate :x (+ (* i pi) pi|2))
      (forward :length 10)
      (right :angle pi|4)
      (left :angle pi|4)
      (forward :length 45)
      (right :angle pi|2)
      (forward :length 40)
      (set :cross-section (m/square 6 140 true))
      (translate :x -1.5)
      (save-transform :frame :board-mount-body :name ::mount-frame)
      (hull
       (forward :length 1)
       (translate :z 5)
       (set :cross-section (m/square 2 140 true))
       (translate :x 2)
       (forward :length 0.01))
      #_(show-coordinate-frame)))))

(let [y1 (.y (.getColumn (lookup-transform board-box ::start-frame) 3))
      y2 (.y (.getColumn (lookup-transform board-box ::mount-frame) 3))]
  (def box-lid-width (+ 4 (* 2 (- y2 y1)))))

(def ^:export-model board-box-lid
  (extrude
   (result :name :board-box-lid
           :expr (union :board-box-lid-body :plate))

   (frame :name :board-box-lid-body
          :cross-section (m/square 1.6 140 true))

   (for [i (range 2)]
     (branch
      :from :board-box-lid-body
      (rotate :x (+ (* i pi) (- pi|2)))
      (forward :length (+ 1.2 (/ box-lid-width 2)))
      (left :angle pi|2 :curve-radius 0.8)
      (forward :length 6.5)
      (translate :x -1/2)
      (set :cross-section (m/square (* 0.8 5) 140 true))
      (translate :x -0.7)
      (loft
       (forward :length 1)
       (translate :z 4 :x 1.0)
       (set :cross-section (m/square 1.6 140 true))
       (forward :length 0.01))))

   (branch
    :from :board-box-lid-body
    :with []
    (frame :cross-section
           (m/difference
            (m/square 50 (- box-lid-width 3.5) true)
            (-> (m/union (m/circle 20/2)
                         (-> (m/square 20 20 true)
                             (m/translate [20/2 0])))
                (m/translate [5 15])))
           :name :plate)
    (translate :z (- 140/2 1.2) :x -25)
    (rotate :z pi)
    (forward :length 1.2)
    (set :cross-section (m/union
                         (-> (m/square 47 1.6 true)
                             (m/translate [0 (- (- (/ box-lid-width 2) 8))]))
                         (m/square 45 1.6 true)
                         (-> (m/square 47 1.6 true)
                             (m/translate [ 0 (- (/ box-lid-width 2) 8)]))))
    (translate :x -2)
    (rotate :x pi)
    (forward :length (+ 4 1.6)))))

(m/union (get-model board-box-lid :board-box-lid)
         (get-model board-box :board-mount))

(def ^:export-model power-switch-box
  (extrude
   (result :name :power-switch-box
           :expr (union (difference :power-switch-box-arm :power-switch-box-mask :bolt-mask)
                        (difference :power-switch-box-body
                                    :power-switch-box-mask)))

   (frame :name :origin)

   (branch
    :from :origin
    :with []

    (frame :cross-section (m/square 65 (+ 27.3 (* 2 1.6)) true)
           :name :power-switch-box-body
           :fn 50)

    (frame :cross-section (m/union
                           (m/square 47 27.3 true)
                           (m/union (for [sign [- +]]
                                      (-> (m/circle 3/2)
                                          (m/translate [(sign 55/2) 0])))))
           :name :power-switch-box-mask)

    (forward :length 32)

    (branch
     :from :origin
     :with [:power-switch-box-mask]
     (set :cross-section (m/square 47 27.3 true))
     (loft
      (forward :length 0.01)
      (translate :z 4)
      (set :cross-section (m/square (- 47 5) (- 27.3 5) true))
      (forward :length 0.01))))

   (branch
    :from :origin
    :with []
    (frame :name :power-switch-box-arm
           :cross-section (m/square 20 4 true))
    (translate :x (- 65/2 10)
               :y (- (/ (+ 27.3 (* 2 1.6)) 2) 2))
    (forward :length 97)
    (branch
     :from :power-switch-box-arm
     :with []
     (frame :name :bolt-mask
            :cross-section (m/circle 7))

     (translate :y -2)
     (forward :y 2)
     (set :cross-section (m/square 11 7 true))
     (forward :y 3))

    (forward :length 15))))

(def tower-support-top-impl
  (let [main-curve-segment
        (fn [i h inner-support?]
          [(translate :x (+ 10.2 3) :z h)
           (rotate :x (+ (* i pi) pi|2))
           (when-not inner-support?
             (branch
              :from :tower-support-top-body
              (translate :x 3)
              (forward :length 29)))
           #_(loft
              (forward :length 0.033)
              (translate :x 3 :z 2.5)
              (forward :length 0.033)
              (forward :length (- 23 5 10))
              (translate :x -3 :z 2.5)
              (forward :length 0.033))
           (forward :length 23.1)
           (when inner-support?
             (branch
              :from :tower-support-top-body
              (forward :length 1.35)
              (left :angle pi|3)
              (forward :length 67)))
           (right :angle pi|6 :curve-radius 3)
           (forward :length 24.58)
           (left :angle pi|2 :curve-radius 3)])]
    (extrude

     (result :name :tower-support-top
             :expr (union :bolt-housing
                          (difference :tower-support-top-body :bolt-mask :vertical-bolt-holes
                                      :horizontal-bolt-holes)))

     (result :name :bolt-housing
             :expr (difference :bolt-mask-body :bolt-mask :tower-support-top-body))

     (frame :cross-section (m/square 6 25 true)
            :name :tower-support-top-body
            :curve-radius 3)

     #_(transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))

     (branch
      :from :tower-support-top-body
      :with []
      (frame :cross-section (m/hull
                             (m/circle 6)
                             (-> (m/square 10 12 true)
                                 (m/translate [5 0])))
             :name :bolt-mask-body)
      (frame :cross-section (m/circle (+ 0.15 5/2))
             :name :bolt-mask)
      (translate :z (- printer-height 10) :x -10.3)
      (rotate :y (- pi|2))
      (forward :length (+ 6 4))
      (set :cross-section (m/cross-section (u/circle-pts 9/2 6)) :to [:bolt-mask])
      (forward :length 5)
      (set :cross-section (m/circle (+ 0.15 5/2)) :to [:bolt-mask])
      (forward :length 30 :to [:bolt-mask]))

     (branch
      :from :tower-support-top-body
      :with []
      (frame :cross-section (m/square 11 7 true)
             :name :vertical-bolt-holes)
      (for [z [(- printer-height 20)
               (- printer-height 45)
               (- printer-height 205)
               (- printer-height 225)]]
        (branch
         :from :vertical-bolt-holes
         (translate :z z)
         (rotate :y pi|2)
         (translate :z 10)
         (forward :length 4)
         (set :cross-section (m/circle 7))
         (forward :length 10))))

     (branch
      :from :tower-support-top-body
      (set :cross-section (m/square 6 240 true))
      (for [i (range 2)]
        (branch
         :from :tower-support-top-body
         (main-curve-segment i (- printer-height 240/2) false)
         (forward :length 66.2)
         (left :angle pi|2))))

     (for [i (range 2)]
       (branch
        :from :tower-support-top-body
        (main-curve-segment i (- printer-height 12.5) true)
        (branch
         :from :tower-support-top-body
         (for [z [10 55]
               y [(if (= i 0) 2.5 -2.5) -217.5 217.5]]
           (branch
            :from :tower-support-top-body :with []
            (frame :cross-section (m/square 11 7 true)
                   :name :horizontal-bolt-holes)
            (translate :z z :y y)
            (rotate :y (- pi|2))
            (forward :length 3.1)
            (rotate :x pi)
            (forward :length 4.1)
            (set :cross-section (m/circle 7))
            (forward :length 20))))

        (forward :length 27.2)
        (branch
         :from :tower-support-top-body
         (left :angle pi|2)
         (forward :length 20)
         (left :angle pi|6 :curve-radius 3)
         (forward :length 50))
        (branch
         :from :tower-support-top-body
         (forward :length (- 66 27))
         (left :angle pi|2 :curve-radius 3)
         (forward :length 20)
         (left :angle pi|6 :curve-radius 3)
         (forward :length 65)))))))

(def ^:export-model tower-support-top
  (m/difference
   (m/transform (get-model tower-support-top-impl :tower-support-top)
                (lookup-transform printer-model :k0.e0/vertical-rod))
   (get-model printer-model :body)
   (get-model printer-model :carriage-mask)))

(def all-tower-top-supports
  (m/difference
   (m/union
    (for [k [:k0.e0/vertical-rod :k0.e1/vertical-rod :k0.e2/vertical-rod]]
      (m/transform (get-model tower-support-top-impl :tower-support-top)
                   (lookup-transform printer-model k))))

   (get-model printer-model :body)))

(def tower-support-top-bolt-housing
  (get-model tower-support-top-impl :bolt-housing))

(def rod-mask-shape
  (m/rotate (m/union
             (m/circle 6.5 50)
             (m/hull
              (-> (m/square 40 5 true)
                  (m/translate [20 -1]))
              (-> (m/square 5 40 true)
                  (m/translate [-4 20])) ) )
            (- T|2)))

(defn hullify [m o]
  (m/hull m (-> m (m/translate [o 0]))))

(def fisheye-mounts
  (extrude
   (result :name :fisheye-mounts
           :expr (difference :fisheye-mount-body-composite :fisheye-mount-mask-composite))

   (result :name :fisheye-mount-mask-composite
           :expr (union :fisheye-mount-mask (mirror :normal [0 1 0] (union :fisheye-mount-mask))))

   (result :name :fisheye-mount-body-composite
           :expr (union :fisheye-mount-body (mirror :normal [0 1 0] (union :fisheye-mount-body))))

   (frame :cross-section (m/difference (m/square 10 20 true)
                                       (-> (m/cross-section (u/circle-pts 10.1 3))
                                           (m/rotate (- T))
                                           (m/translate [0 -13])))
          :name :fisheye-mount-body)

   (frame :cross-section (hullify (m/circle 7/2 50) 7)
          :name :fisheye-mount-mask)

   (rotate :z pi|2)

   (translate :x (/ 65 2))
   (rotate :y (- pi|2))
   (translate :x 2 :to [:fisheye-mount-mask])
   (forward :length (- outer-mount-length 2.4))
   (set :cross-section (hullify (m/circle (+ 0.1 3/2) 50) 7) :to [:fisheye-mount-mask])
   (forward :length 2.4)
   (set :cross-section rod-mask-shape :to [:fisheye-mount-mask])
   (forward :length (+ 3 (* 1 carriage-spacer-length)))
   (forward :length (+ 3 (* 1 carriage-spacer-length)))
   (set :cross-section (hullify (m/circle (+ 0.1 3/2) 50) 7) :to [:fisheye-mount-mask])
   (forward :length 3.2)
   (set :cross-section (m/square 6 12 true) :to [:fisheye-mount-mask])
   (forward :length (- 7.5 1.6))))

(def belt-mask-polygon
  (let [c 152
        r (/ c (* 2 pi))
        n-teeth (/ 152 2)
        pts (map (fn [[x y]]
                   [(+ x 1) y])
                 (u/cos-wave-points 2 1.0 n-teeth))
        n-pts (count pts)
        a (/ (* 2.0 pi) n-pts)]
    (m/square 100 2.15 true)))

(def ^:export-model carriage-mount
  (extrude
   (result :name :carriage-mount
           :expr (union
                  (difference :fisheye-mounts :belt-mask)
                  (difference :carriage-mount-body :bolt-masks :belt-mask :fisheye-mount-mask-composite :endstop-screw)))

   (frame :cross-section (m/square 65 65 true)
          :name :carriage-mount-body)

   (branch
    :from :carriage-mount-body
    :with []
    (frame :name :origin)
    (translate :z 13)
    (:forms fisheye-mounts))

   (branch
    :from :carriage-mount-body
    :with []
    (frame :name :endstop-screw
           :cross-section (m/circle (/ 1.8 2) 20))
    (translate :y (- (- 65/2 21.5)) :z 4)
    (rotate :y (- pi|2))
    (translate :z 10)
    (forward :length 100))

   (frame :cross-section (m/union
                          (for [x [-20 20]
                                y [-20 20]]
                            (->> (for [dx [-2 2]]
                                   (-> (m/circle (+ 0.6 5/2) 30)
                                       (m/translate [(+ x dx) y ])
                                       #_(m/rotate (+ -4 (* (/ 8 20) i)))))
                                 (partition 2 1)
                                 (map m/hull)
                                 (m/union))))
          :name :bolt-masks)

   (forward :length 5)
   (offset :delta 3 :to [:bolt-masks])
   (forward :length 5)

   (frame :name :belt-mask
          :cross-section (m/union (-> belt-mask-polygon
                                      (m/translate [0 (- (- (/ 13.5 2) 0.7))]))
                                  (-> belt-mask-polygon
                                      (m/translate [0 (- (- (/ 13.5 2) 6))])
                                      (m/rotate T))
                                  (-> (m/square 19 8 true)
                                      (m/translate [0 (- 2.7)]))
                                  (-> (m/square 100 4 true)
                                      (m/translate [0 (- (- (/ 13.5 2) 0.7))])
                                      (m/rotate T))) )

   (set :cross-section (m/union
                        (m/square 45 25 true)
                        (let [x (m/hull (-> (m/square 45 3 true)
                                            (m/translate [0 2.9]))
                                        (-> (m/square 3 0.2 true)
                                            (m/translate [(+ 45/2 3) 2.9])))]
                          (m/union x (m/mirror x [-1 0 0])))
                        (m/translate (m/hull (m/square 45 5.8 true)
                                             (-> (m/square 3 0.2 true)
                                                 (m/translate [(+ 45/2 3) 0])))
                                     [0 -9.65])
                        (m/translate (m/hull (m/square 45 5.8 true)
                                             (-> (m/square 3 0.2 true)
                                                 (m/translate [(- (+ 45/2 3)) 0])))
                                     [0 -9.65])

                        (m/hull (-> (m/square 45 5 true)
                                    (m/translate [0 -2.5]))
                                (-> (m/square 3 0.2 true)
                                    (m/translate [(+ 45/2 3) -3])))
                        (m/hull (-> (m/square 45 5 true)
                                    (m/translate [0 -2.5]))
                                (-> (m/square 3 0.2 true)
                                    (m/translate [(- (+ 45/2 3)) -2]))))
        :to [:carriage-mount-body])
   (forward :length 6)
   (forward :length 8 :to [:belt-mask])))

(def ^:export-model split-carriage-mount
  (let [[left right] (m/split-by-plane (get-model carriage-mount :carriage-mount)
                                       [0 1 0] 8.05)]
    (m/union (m/translate left [0 1 0])
             right)))

(def tower-support-bolt-holes
  (extrude
   (result :name :tower-support-bolt-holes :expr :side-rod-bolt-point)
   (frame :name :side-rod-bolt-point :cross-section (m/square 7 11 true))
   (for [tf-name [:k0.e0.r0/side-rod :k0.e0.r1/side-rod
                  :k0.e1.r0/side-rod :k0.e1.r1/side-rod
                  :k0.e2.r0/side-rod :k0.e2.r1/side-rod]]
     (branch
      :from :side-rod-bolt-point
      (transform :replace (lookup-transform printer-model tf-name))
      (translate :z 10)
      (for [sign [+ -]]
        (branch
         :from :side-rod-bolt-point
         (translate :y (sign 360/2))
         (rotate :y pi|2)
         (for [x [0 (- (sign 25))]]
           (branch
            :from :side-rod-bolt-point
            (translate :y x)
            (forward :length 13)
            (set :cross-section (m/circle 14/2))
            (forward :length 50)))))))))

(def tower-support
  (extrude
   (result :name :tower-support
           :expr
           (difference
            :tower-support-body
            :tower-support-mask
            :stepper-bolt-mask
            :stepper-shaft-hole
            :tower-support-bolt-holes
            :vertical-rod-bolt-holes))

   (frame :cross-section (m/square 10 10 true) :name :tower-support-body)
   (branch :from :tower-support-body :with [] (:forms tower-support-bolt-holes))
   #_(transform :replace (lookup-transform printer-model :k0.e1/vertical-rod))
   (branch
    :from :tower-support-body :with []
    (frame :name :vertical-rod-bolt-holes :cross-section (m/square 11 7 true))
    (segment
     (for [z [8 30 165 185]]
       (branch
        :from :vertical-rod-bolt-holes
        (translate :x 10 :z z)
        (rotate :y (+ pi|2))
        (forward :length 4)
        (set :cross-section (m/circle 13/2))
        (forward :length 20)))))

   (branch
    :from :tower-support-body :with []
    (frame :name :tower-support-mask :cross-section (m/square 20 20 true))
    (translate :x -10)
    (branch
     :from :tower-support-mask
     (translate :x (- stepper-mount-frame-offset))
     (rotate :y (- pi|2))
     (set :cross-section (m/square 60 60 true))
     (translate :x 30)
     (branch
      :from :tower-support-mask
      :with []
      (frame :name :stepper-bolt-mask
             :cross-section (m/union (for [x [15.5 -15.5]
                                           y [15.5 -15.5]]
                                       (-> (m/circle 1.6)
                                           (m/translate [x y])))) )
      (frame :name :stepper-shaft-hole
             :cross-section (m/circle 11.2))
      (rotate :y pi)
      (forward :length 4)
      (offset :delta 6)
      (forward :length 8 :to [:stepper-shaft-hole])
      (forward :length 200 :to [:stepper-bolt-mask]))
     (forward :length 200)))

   (branch
    :from :tower-support-body :with []
    (frame :name :tower-support-mask :cross-section (m/square belt-shaft-width belt-shaft-width true))
    (translate :x (- (+ 10 belt-shaft-mask-offset)))
    (forward :length 200))

   (hull
    (for [sign [-1 1]
          [height length] [[5 62.5]
                           [60 62.5]
                           [190 10]]]
      [(transform :replace (m/frame 1) #_(lookup-transform printer-model :k0.e1/vertical-rod))
       (translate :z height :x 85)
       (rotate :z (* sign pi|6))
       (rotate :y (- pi|2))
       (translate :z 80)
       (forward :length length)
       (rotate :x (* sign (- (+ pi|6 pi|3))))
       (forward :length 10)]))))

(def ^:export-model base-tower-support
  (m/difference (m/transform (get-model tower-support :tower-support)
                             (lookup-transform printer-model :k0.e1/vertical-rod))
                (get-model tower-support :tower-support-bolt-holes)
                (get-model printer-model :carriage-mask)
                (get-model printer-model :side-rod-mask)
                (get-model printer-model :body)
                (get-model printer-model :heated-bed-mask)))

(def all-base-tower-supports
  (m/difference (m/union
                 (for [k [:k0.e0/vertical-rod :k0.e1/vertical-rod :k0.e2/vertical-rod]]
                   (m/transform
                    (get-model tower-support :tower-support)
                    (lookup-transform printer-model k))))
                (get-model printer-model :carriage-mask)
                (get-model printer-model :side-rod-mask)
                (get-model printer-model :body)
                (get-model printer-model :heated-bed-mask)))

(def ^:export-model build-plate-support-2
  (extrude

   (result :name :build-plate-support
           :expr (difference :build-plate-support-body
                             :bolt-mask
                             :build-plate-mask))

   (frame :cross-section (m/square 6 165 true)
          :name :build-plate-support-body)

   (branch
    :from :build-plate-support-body :with []
    (:forms build-plate-mask))

   (transform :replace (lookup-transform printer-model :k0.e0.r1/side-rod))
   (translate :z 23)

   (branch
    :from :build-plate-support-body
    (translate :x -10)
    (rotate :y pi|2)
    (forward :length 25)
    #_(show-coordinate-frame))

   (branch
    :from :build-plate-support-body
    :with []
    (frame :cross-section (m/hull
                           (-> (m/circle 2)
                               (m/translate [-2 0]))
                           (-> (m/circle 2)
                               (m/translate [2 0])))
           :name :bolt-mask)
    (for [y-off [(- 75) 75]]
      (branch
       :from :bolt-mask
       (translate :y y-off :z -3)
       (forward :length 1.2)
       (offset :delta 2)
       (forward :length 5)
       #_(show-coordinate-frame))))))

(def ^:export-model adjustable-build-plate-support
  (extrude
   (result :name :adjustable-build-plate-support
           :expr (difference :build-plate-support-body
                             :bolt-mask
                             (-> (m/cylinder 30 3/2 3/2 20)
                                 (m/rotate [T|2 0 0])
                                 (m/translate [0 12 10]))))

   (frame :name :build-plate-support-body
          :cross-section (m/square 40 20 true))
   (frame :name :bolt-mask
          :cross-section (m/union (-> (m/circle 3/2 40)
                                      (m/translate [12 0]))
                                  (-> (m/circle 3/2 40)
                                      (m/translate [-12 0]))))

   (hull
    :to [:build-plate-support-body]
    (forward :length 3)
    (offset :delta 3 :to [:bolt-mask])
    (set :cross-section (m/square 20 20 true) :to [:build-plate-support-body])
    (forward :length 17))))

(let [slider-shape (m/hull
                    (-> (m/circle 2 20)
                        (m/translate [-11 0]))
                    (-> (m/circle 2)
                        (m/translate [11 0])))
      w 40 l 14 h 20]
  (def ^:export-model adjustable-build-plate-side-support
    (m/difference
     (-> (m/square w l true)
         (m/extrude h))
     (-> (m/square (- w 3.2) (- l 3.2) true)
         (m/extrude (- h 2)))
     (-> slider-shape
         (m/extrude (+ 2 h)))
     (-> (m/circle 3/2)
         (m/extrude 50)
         (m/rotate [0 T|2 0])
         (m/translate [0 0 10]))
     (-> (m/circle 5/2)
         (m/extrude 50)
         (m/rotate [0 (- T|2) 0])
         (m/translate [0 0 10]))))

  (def ^:export-model adjustable-build-plate-side-support-slider
    (extrude
     (result :name :adjustable-build-plate-side-support-slider
             :expr (difference :body :mask))
     (frame :name :body :cross-section (m/square 18 14 true))
     (frame :name :mask :cross-section (m/circle 3))
     (forward :length 3)
     (set :cross-section (m/circle 3/2) :to [:mask])
     (forward :length 1.2))))

(def ^:export-model build-plate-support-3
  (extrude
   (result :name :support
           :expr (difference :body :mask))
   (frame :name :body
          :cross-section (m/square 40 20 true))
   (frame :name :mask
          :cross-section (m/hull (for [sign [+ -]]
                                   (-> (m/circle (+ 0.1 3/2))
                                       (m/translate [(sign 12) 0])))))
   (forward :length 1.6)))

(def ^:export-model carriage-heat-shield-mount
  (extrude
   (result :name :carriage-heat-shield-mount
           :expr (difference :body :mask))

   (frame :name :body
          :cross-section (-> (m/square 9 18 true)
                             (m/offset 1/2 :round -1 30)))

   (frame :name :mask
          :cross-section (m/circle 4/2 30))

   (translate :y 3 :to [:mask])
   (loft
    :to [:body]
    (forward :length 3)
    (offset :delta 1.5 :to [:mask])
    (loft
     :to [:mask]
     (forward :length 0.01 :to [:mask])
     (translate :z 15)
     (forward :length 0.01)))
   (forward :length 6)
   (to
    :frames [:mask]
    (set :cross-section (m/circle 3/2 30))
    (translate :y -8)
    (rotate :x pi)
    (forward :length 10))))

(def ^:export-model endstop-mount
  (extrude
   (result :name :endstop-mount
           :expr (difference (hull :body (->> :heat-shield-mount/carriage-heat-shield-mount
                                              (translate {:x 28/2 :y 3.5})))
                             :endstop-bolt-mask
                             (-> (m/cylinder 100 3/2)
                                 (m/translate [28/2 6 22]))
                             (-> (m/cube 18 20 8 true)
                                 (m/translate [0 0 40]))
                             (-> (m/cube 20 20 100 true)
                                 (m/rotate [90 0 0])
                                 (m/translate [28/2 0 10]))
                             :side-bolt-mask))
   (frame :name :body
          :cross-section (m/square 8 19 true))

   (branch
    :from :body
    :with []
    (frame :name :side-bolt-mask
           :cross-section (m/circle 3/2))
    (translate :z 10 :y 2 :x 4)
    (rotate :y (- pi|2))
    (forward :length 1.6)
    (set :cross-section (m/circle 3))
    (forward :length 30))

   (frame :name :endstop-bolt-mask
          :cross-section (m/union
                          (-> (m/circle 3/2)
                              (m/translate [28/2 0]))
                          (-> (m/circle 3/2)
                              (m/translate [-28/2 0]))))
   (translate :y -3.5 :to [:endstop-bolt-mask])
   (forward :length 17)
   (set :cross-section (m/square 37 19 true) :to [:body])
   (translate :x 1/2 :to [:body])
   (forward :length 3)

   (insert :extrusion carriage-heat-shield-mount
           :models [:mask :carriage-heat-shield-mount]
           :ns :heat-shield-mount)

   (forward :length 1)
   (to
    :models [:endstop-bolt-mask]
    (set :cross-section (m/square 36 12 true))
    (forward :length 40))))

(def fan-w 40)
(def fan-l 10)
(def fan-h 40)
(def heat-sink-r 12)
(def joint-r1 6)
(def joint-r2 8)
(def heat-sink-height 43)
(def heat-block-width 11.5)
(def heat-block-length 24)
(def heat-block-height 20)
(def heat-block-offset-x (/ (- 4 (/ 11.5 2)) 2))
(def heat-block-offset-y (/ (- 8 (/ 24 2)) 2))

(def heat-break-height 20)

(def mag-r 6)

(def opening-mask
  (m/union (m/circle (+ heat-sink-r 1.5))
           (m/square (+ heat-block-width 3)
                     (+ heat-block-length 3))))

(defn make-opening-shape [offset]
  (let [w (+ fan-w 8)
        side-length w]
    (m/cross-section
     (butlast
      (points
       :axes [:x :y]
       (frame :name :origin :curve-radius (- 3 offset))
       (rotate :z (+ pi|3 pi|6))
       (translate :x offset)
       (rotate :x pi|2)
       (forward :length (/ w 2))
       (right :angle (+ pi|2 pi|6))
       (forward :length side-length)
       (right :angle (+ pi|2 pi|6))
       (forward :length side-length)
       (right :angle (+ pi|2 pi|6))
       (forward :length side-length))))))

(def heatbreak-body-length 26.2)
(def heatbreak-body-radius (/ 22.2 2))
(def heatbreak-joint-offset 4.2)
(def heatbreak-joint-radius-1 8.2)
(def heatbreak-joint-radius-2 5.8)
(def heatbreak-joint-length 6)
(def heatbreak-gap-length 3)
(def fan-radius (/ 37.5 2))

(def coupling-shape
  (m/translate
   (m/hull (-> (m/square (+ heatbreak-joint-radius-1 33) 1 true)
               (m/translate [0 1/2]))
           (-> (m/square (+ heatbreak-joint-radius-1 33) 1 true)
               (m/translate [0 22])))
   [0 -8]))

(def heat-block-sock-skin-width 1.5)

(def heat-block-model
  (extrude
   (result :name :heat-block-model :expr :heat-block)
   (frame :cross-section (m/square (+ heat-block-width (* 2 heat-block-sock-skin-width))
                                   (+ heat-block-length (* 2 heat-block-sock-skin-width))
                                   true)
         :name :heat-block)
   (translate :x heat-block-offset-x
              :y heat-block-offset-y)
   (forward :length heat-block-height)))

(def extruder-mask
  (extrude
   (result :name :extruder-mask
           :expr (union :extruder-mask-heatbreak :heatbreak-joint-mask :extruder-fan-mask
                        :cable-slot :fan-bolt-holes :coupling-bolt-mask))
   (frame :cross-section (-> (m/square (+ heat-block-width 7.5)
                                       (+ heat-block-length 6)
                                       true)
                             (m/translate [heat-block-offset-x
                                           heat-block-offset-y])
                             (m/union (m/hull (m/circle (+ heatbreak-body-radius 2) 60)
                                              (-> (m/square 24 10 true)
                                                  (m/translate [0 -17])))))
          :name :extruder-mask-heatbreak)
   (forward :length heat-block-height)
   (forward :length (- (+ heatbreak-gap-length (/ heatbreak-body-length 2))
                       heatbreak-gap-length))
   (set :cross-section (m/circle (+ heatbreak-body-radius 2) 50))
   (forward :length heatbreak-gap-length)

   (branch
    :from :extruder-mask-heatbreak
    :with []
    (frame :cross-section (m/circle 4) :name :cable-slot)
    (translate :y (- (+ 13 (/ heatbreak-body-radius 2))))
    (forward :length 40))

   (branch
    :from :extruder-mask-heatbreak
    :with []
    (frame :cross-section (m/circle fan-radius 150) :name :extruder-fan-mask)
    (rotate :x (- pi|2))
    (translate :z (- (+ 20 heatbreak-body-radius)))

    ;; Mount bolt holes
    (for [rot [(+ pi|4 pi) (+ pi|4 pi|2 pi) (+ pi|4 pi pi) (+ pi|4 pi pi pi|2)]]
      (branch
       :from :extruder-fan-mask
       :with []
       (frame :cross-section (m/circle 3/2) :name :fan-bolt-holes)
       (rotate :z rot)
       (translate :x (+ 2 2.1 fan-radius))
       (forward :length 25)))

    (hull
     (hull
      (forward :length 20)
      (translate :z (+ 2 heatbreak-body-radius))
      (set :cross-section (u/ovol (/ (* 2 (+ heatbreak-body-radius 2))
                                     2)
                                  (/ (+ 4 heatbreak-body-length)
                                     2)))
      (forward :length 1))
     (translate :z (+ 2 heatbreak-body-radius))
     (set :cross-section (u/ovol (/ (* 2 (- heatbreak-body-radius 1.5))
                                    2)
                                 (/ (+ 15 heatbreak-body-length)
                                    2)) )
     (translate :y 4)
     (forward :length 1))
    (forward :length 20))

   (branch
    :from :extruder-mask-heatbreak
    :with []
    (frame :name :heatbreak-joint-mask
           :cross-section (m/circle (+ heatbreak-body-radius 2) 50))
    (forward :length (/ heatbreak-body-length 2))
    (loft
     (forward :length 0.01)
     (set :cross-section (m/circle heatbreak-joint-radius-1 50))
     (translate :z heatbreak-joint-offset)
     (forward :length 0.01))
    (forward :length 3)
    (set :cross-section (m/circle heatbreak-joint-radius-2 50))

    (branch
     :from :heatbreak-joint-mask
     (rotate :x (- pi|2))
     (rotate :z pi)
     (save-transform :frame :heatbreak-joint-mask :name ::coupling-junction))

    (forward :length heatbreak-joint-length)
    (set :cross-section (m/circle heatbreak-joint-radius-1))
    (forward :length 2)
    (save-transform :frame :heatbreak-joint-mask :name ::bolt-hole)

    (branch
     :from :heatbreak-joint-mask
     :with []
     (frame :name :coupling-bolt-mask)
     (translate :z (- 5))
     (rotate :x pi|2)
     (for [rot [0 pi]]
       (branch
        :from :coupling-bolt-mask
        (set :cross-section (m/union
                             (m/translate (m/circle (+ 1/4 3/2)) [11 0])
                             (m/translate (m/circle (+ 1/4 3/2)) [-11 0]))
             :fn 6)
        (rotate :y rot)
        (forward :length 4)
        (set :cross-section (m/union
                             (m/translate (m/circle 6/2 6) [11 0])
                             (m/translate (m/circle 6/2 6) [-11 0])))
        (forward :length 60))))

    (forward :length 2))))

(def coupling-cube
  (extrude
   (result :name :coupling-cube :expr (difference :body :bolt-mask))
   (frame :name :origin :fn 50)
   (branch
    :from :origin
    (frame :cross-section (m/square 20 20 true) :name :body)
    (forward :length 20))
   (branch
    :from :origin
    (frame :cross-section (m/circle 5) :name :bolt-mask)
    (translate :z 10)
    (for [i (range 2)]
      (branch
       :from :bolt-mask
       (rotate :x (+ pi|2 (* i pi|2)))
       (translate :z -10)
       (forward :length (- 20 1.6))
       (set :cross-section (m/circle 1.6))
)))))

(def bltouch-mask-shape
  (m/hull (m/circle 7 6)
          (-> (m/square 14 7 true)
              (m/translate [0 -5.0]))))

(def bltouch-top-shape
  (m/hull (-> (m/circle (+ 2.5 3/2))
              (m/translate [-9 0]))
          (m/square 8 11.5 true)
          (-> (m/circle (+ 2.5 3/2))
              (m/translate [9 0]))))

(def bltouch-model
  (extrude
   (result :name :bltouch-model :expr :bltouch-body)
   (frame :cross-section (m/circle 1) :name :bltouch-body :fn 60)
   (forward :length 5)
   (set :cross-section (m/circle 3))
   (hull
    (forward :length 0.01)
    (set :cross-section bltouch-mask-shape)
    (translate :z 1.98)
    (forward :length 0.01))
   (forward :length 33)
   (set :cross-section bltouch-top-shape)
   (forward :length 2)))

(def abl-mask
  (let [angle (/ pi 7)]
    (extrude
     (result :name :abl-mask :expr (difference :auto-bed-level-body :auto-bed-level-mask))
     (frame :cross-section bltouch-mask-shape :name :auto-bed-level-mask)
     (frame :cross-section bltouch-top-shape :name :auto-bed-level-body)

     (rotate :z (- (+ pi|6 pi|2)))
     (translate :y 10.0 :z -2)
     (rotate :x (- angle))
     (forward :z 9.0 :gap [:auto-bed-level-body])
     (forward :length 29.5)
     (forward :length 0.01 :to [:auto-bed-level-mask])
     (set :cross-section (m/union bltouch-mask-shape bltouch-top-shape) :to [:auto-bed-level-mask])
     (translate :z -0.025)
     (forward :length 10 :to [:auto-bed-level-mask] :branch? true)
     (to :models [:auto-bed-level-mask]
         (rotate :x pi)
         (translate :z 9.9)
         (set :cross-section (m/union
                              (-> (m/circle 3/2)
                                  (m/translate [9 0]))
                              (-> (m/circle 3/2)
                                  (m/translate [-9 0]))))
         (forward :length 15)))))

(def extruder-assembly-body-fisheye-mechanical-base
  (extrude
   (result :name :extruder-assembly-body-fisheye-mechanical-base
           :expr
           (difference
            (hull (union :extruder-assembly-base-body))
            (union
             (for [i (range 3)]
               (->> (union :extruder-assembly-base-mask)
                    (translate :x 39)
                    (rotate :z (* i (* 2/3 pi))))))))

   (frame :cross-section opening-mask :name :extruder-assembly-base-mask)
   (frame :cross-section (m/circle 45 3) :name :extruder-assembly-base-body)

   ;; base hexagon
   (set :cross-section (m/square 0.1 35 true)
        :to [:extruder-assembly-base-body])
   (set :cross-section (m/square 35 17 true)
        :to [:extruder-assembly-base-mask])

   (for [i (range 3)]
     (branch
      :from :extruder-assembly-base-body
      :with [:extruder-assembly-base-body]
      (rotate :z (* i 2/3 pi))
      (translate :x 39)
      (forward :length 13 :to [:extruder-assembly-base-body])))

   (branch
    :from :extruder-assembly-base-mask
    :with [:extruder-assembly-base-mask]
    (set :cross-section (-> (m/circle 3/2)
                            (m/translate [0 8])))
    (translate :z 17.7 :x -4)
    (rotate :x (- pi|2))
    (save-transform :frame :extruder-assembly-base-mask :name ::rod-mount-holes)
    (forward :length 100 :center true))

   (to
    :models [:extruder-assembly-base-mask]
    (set :cross-section (m/difference
                         (m/square 26 60 true)
                         (m/square 26 30 true)))
    (forward :length 13.5))))

(def extruder-assembly-body-fisheye
  (extrude
   (result :name :extruder-assembly-body-fisheye
           :expr
           (union (difference :auto-bed-level-body
                              :auto-bed-level-mask
                              :extruder-mask-heatbreak
                              :fan-hole-mask)
                  (difference :center-triangle-body
                              :auto-bed-level-mask
                              :extruder-mask)
                  (difference (union :extruder-assembly-body-fisheye-mechanical-base :fan-hole-body)
                              :fan-hole-mask
                              :auto-bed-level-mask
                              :extruder-mask-heatbreak)))
   (frame :name :origin)

   (branch
    :from :origin
    (rotate :z (- pi|6))
    (:forms extruder-mask))

   (branch
    :from :origin
    (:forms abl-mask))

   (branch
    :from :origin
    (:forms extruder-assembly-body-fisheye-mechanical-base))

   (branch
    :from :origin
    (frame :name :fan-hole-body)
    (frame :name :fan-hole-mask)

    (for [i (range 1 2)]
      (branch
       :from :fan-hole-body
       :with [:fan-hole-body :fan-hole-mask]
       (set :cross-section (m/square 7 22 true)
            :to [:fan-hole-body])
       (set :cross-section (m/square 5 20 true)
            :to [:fan-hole-mask])
       (rotate :z (* i (/ (* 2 pi) 3)))
       (translate :x 18.0)
       (loft
        (forward :length 1/2)
        (translate :x 8 :z 4)
        (forward :length 1/2))
       (loft
        (forward :length 1/2)
        (translate :x -1.6 :z 8)
        (when (zero? i)
          (translate :y 6))
        (set :cross-section (m/square (- 15.2 (* 2 1.3) 2.2) (- 19.2 (* 2 1.3) 2.2) true) :to [:fan-hole-mask])
        (set :cross-section (m/square (- 15.2 2.5) (- 19.2 2.5) true) :to [:fan-hole-body])
        (forward :length 0.1))
       (if (zero? i)
         (forward :length 12.2)
         (forward :length 1.5))
       (when (zero? i)
         (up :angle pi|4 :curve-radius 22/2))
       (forward :length 0.01 :to [:fan-hole-mask])
       (forward :length 6))))

   (branch
    :from :origin
    (rotate :z (- pi|6))
    (frame :cross-section (make-opening-shape 0)
           :name :center-triangle-body)
    (frame :cross-section (m/circle 4)
           :name :tmp-mask)
    (translate :y -18.3 :to [:center-triangle-body])
    (forward :length (+ 26 fan-h))
    (save-transform :frame :center-triangle-body :name :extruder-assembly-top))))

(def coupling-mask-segment
  (extrude
   (result :name :coupling-mask-segment :expr :coupling-mask)
   (frame :cross-section coupling-shape :name :coupling-mask)
   (to
    :models [:coupling-mask]
    (transform :replace (lookup-transform extruder-mask ::coupling-junction))
    (rotate :y (- pi|6))
    (forward :length 50))))

(def ^:export-model extruder-assembly
  (extrude
   (result :name :extruder-assembly
           :expr (difference :extruder-assembly-body-fisheye
                             :coupling-mask-segment))
   (insert :extrusion extruder-assembly-body-fisheye)
   (insert :extrusion coupling-mask-segment)))

(def ^:export-model extruder-coupling
  (extrude
   (result :name :extruder-coupling
           :expr (difference (intersection :center-triangle-body
                                           :coupling-mask-segment)
                             :extruder-mask))
   (insert :extrusion extruder-assembly-body-fisheye
           :models [:center-triangle-body :extruder-mask])
   (insert :extrusion coupling-mask-segment)))

(def ^:export-model full-printer-model
  (m/union (get-model printer-model :printer-model)
           all-base-tower-supports
           all-tower-top-supports))

(def bolt-segment
  (extrude
   (result :name :bolt-segment
           :expr (difference :bolt-segment-body :bolt-segment-mask))
   (frame :name :bolt-segment-body :cross-section (m/square 20 20 true))
   (frame :name :bolt-segment-mask :cross-section (m/circle 3/2))
   (rotate :z (- pi|2))
   (hull
    :to [:bolt-segment-body]
    (forward :length 3)
    (to
     :models [:bolt-segment-mask]
     (set :cross-section (m/circle 5.5))
     (forward :length 20))
    (to
     :models [:bolt-segment-body]
     (set :cross-section (m/square 20 0.1 true))
     (translate :y 10 :z 20)
     (forward :length 1)))))

(def bolt-segment-mask
  (extrude
   (result :name :bolt-mask :expr (union :bolt-mask-body))
   (frame :name :bolt-mask-body :cross-section (m/circle 3/2))
   (rotate :z (- pi|2))
   (forward :length 3)
   (set :cross-section (m/circle 5.5))
   (forward :length 70)))

(def side-panel-support
  (extrude
   (result :name :side-panel-support
           :expr (difference :body :mask))

   (frame :name :body :cross-section (m/square 80 20 true))
   (frame :name :mask
          :cross-section (m/union (-> (m/circle 3/2)
                                      (m/translate [32 0]))
                                  (-> (m/circle 3/2)
                                      (m/translate [-32 0]))))
   (forward :length 2)
   (offset :delta 3 :to [:mask])
   (forward :length 30)
   (branch :from :mask :with [:mask] (forward :length 100))
   (forward :length 1)
   (up :angle pi|3 :curve-radius 20)
   (forward :length 10)
   (offset :delta -3 :to [:mask])
   (forward :length 2)
   (to :models [:mask]
       (rotate :y pi)
       (forward :length 2)
       (offset :delta 3)
       (forward :length 200))))

(defn make-frame-support [result-name left-right-center? X Y]
  (let [Xpos (subvec (tf/translation-vector X) 0 2)
        Xdir (vec (take 2 (.getColumn X 2)))
        Ypos (subvec (tf/translation-vector Y) 0 2)
        Ydir (vec (take 2 (.getColumn Y 2)))
        curve-radius 20

        Zdir (e/- Ypos Xpos)

        A (acos (/ (dot-product Xdir Ydir)
                   (* (magnitude Xdir) (magnitude Ydir))))
        C (acos (/ (dot-product Zdir Xdir)
                   (* (magnitude Zdir) (magnitude Xdir))))
        B (- pi A C)

        a (magnitude Zdir)
        b (/ (* a (sin B))
             (sin A))
        c (/ (* a (sin C))
             (sin A))

        y-scalar (/ b (magnitude Ydir))

        D (- pi A)
        x (* c (sin (/ A 2)))
        y (/ x (sin (/ D 2)))

        cr 20
        forward-length (- (magnitude Zdir) (* 2 cr (sin (/ D 2))))]
    (extrude
     (result :name result-name
             :expr (union #_:bolt-segment (difference :frame-support-body :bolt-mask)))
     (frame :name :origin)

     (transform :replace Y)
     (translate :x -200)
     (branch
      :from :origin
      (insert :extrusion bolt-segment-mask
              :models [:bolt-mask]
              :end-frame :bolt-mask-body))
     (let [beam-height 40]
       (branch
        :from :origin
        (frame :name :frame-support-body :cross-section (m/square beam-height 20 true))
        (for [op (case left-right-center? :left [up] :right [down] :center [down up])]
          (branch
           :from :frame-support-body
           (op :angle (/ D 2) :curve-radius cr :cs 200)
           (forward :length forward-length)
           (op :angle (/ D 2) :curve-radius cr :cs 200)
           #_(let [l (- (* (magnitude Xdir) y-scalar) c)]
               (when (> l 1)
                 (forward :length l)))
           (rotate :x (- pi))
           (rotate :z pi|2)
           (insert :extrusion bolt-segment-mask
                   :models [:bolt-mask]
                   :end-frame :bolt-mask-body))))))))

(def printer-center-offset -440)

(def printer-1-tf
  (-> (m/frame 1) (m/rotate [0 0 pi|3]) (m/translate [printer-center-offset 0 0])))

(def printer-2-tf
  (-> (m/frame 1) (m/translate [printer-center-offset 0 0])))

(def printer-3-tf
  (-> (m/frame 1) (m/rotate [0 0 (- pi|3)]) (m/translate [printer-center-offset 0 0])))

(def ^:export-model left-frame-support
  (make-frame-support :left-frame-support :left
                      (m/compose-frames
                       printer-1-tf
                       (lookup-transform printer-model :k0.e1/frame-support-mount))
                      (m/compose-frames
                       printer-2-tf
                       (lookup-transform printer-model :k0.e2/frame-support-mount))))

#_(let [m (get-model printer-model :body)
      bounds (.Center (m/bounds m))
      ret
      (m/union (m/center m)
               (-> (get-model printer-model :heated-bed-mask)
                   (m/translate [(- (.x bounds)) 0 0])))]
  (m/union ret (-> ret
                   (m/rotate [0 0 T])
                   (m/translate [40 430 0]))))

(def ^:export-model linear-side-support
  (extrude
   (result :name :linear-side-support
           :expr (difference :body :mask))
   (frame :name :origin)

   (branch
    :from :origin :with []
    (frame :name :mask :cross-section (m/circle 3/2))
    (rotate :y pi|2)
    (translate :z -10 :x -10)
    (for [y [-10 10]]
      (branch
       :from :mask
       (translate :y y)
       (forward :length 2)
       (set :cross-section (m/circle 5))
       (forward :length 20))))

   (branch
    :from :origin :with []
    (frame :name :body :cross-section (m/square 20 40 true))
    (forward :length 210)
    (left :angle pi|3 :curve-radius 10)
    (branch
     :from :body
     :with []
     (frame :name :mask :cross-section (m/circle 3/2))
     (rotate :x pi)
     (forward :length 2)
     (set :cross-section (m/circle 5))
     (forward :length 50)))))

(def radial-print-farm
  (m/union (m/transform (get-model printer-model :body) printer-1-tf)
           #_(m/transform (get-model printer-model :heated-bed-mask) printer-1-tf)
           (m/transform (get-model printer-model :body) printer-2-tf)
           #_(m/transform (get-model printer-model :heated-bed-mask) printer-2-tf)
           (m/transform (get-model printer-model :body) printer-3-tf)
           (get-model left-frame-support :left-frame-support)))

#_(def radial-print-farm
  (m/union (m/union (for [i (range 6)]
                      (-> full-printer-model
                          (m/rotate [0 0 (* i T|3)]))))
           (let [support (get-model left-frame-support :left-frame-support)]
             (m/union
              (for [i (range 6)
                    z [20 600]]
                (-> support
                    (m/rotate [0 0 (* i T|3)])
                    (m/translate [0 0 z])))))
           (-> (m/union
                (for [i [-1 1]]
                  (-> (m/square panel-width panel-height true)
                      (m/translate [(* i (/ panel-width 2)) 0]))))
               (m/extrude panel-thickness)
               (m/translate [0 0 printer-height]))))

#_(def right-frame-support
  (make-frame-support :right-frame-support :right))

#_(def center-frame-support
  (make-frame-support :center-frame-support :center))

(def ^:export-model extruder-mold
  (extrude
   (result :name :extruder-mold
           :expr (difference :body :mask))

   (frame :name :mask :cross-section (m/circle (+ 0.1 4/2) 100))
   (frame :name :body :cross-section (m/circle 9/2 100))
   (forward :length 1.2)
   (set :cross-section (m/circle (- 9/2 0.8) 100) :to [:mask])
   (forward :length 10)
   (hull
    (forward :length 1)
    (set :cross-section (m/circle 2 100) :to [:body])
    (set :cross-section (m/circle 1 100) :to [:mask])
    (translate :z 4)
    (forward :length 0.1))))

(def ^:export-model extruder-mold-with-thermistor
  (let [outer-shape (m/hull (-> (m/circle (/ 10 2) 100)
                                (m/translate [-2 0]))
                            (-> (m/circle (/ 8 2) 100)
                                (m/translate [2 0])))]
    (extrude
     (result :name :extruder-mold-with-thermistor
             :expr (difference :body :feed-mask :thermistor-mask))

     (frame :name :body
            :cross-section outer-shape)
     (frame :name :feed-mask :cross-section (m/circle (+ 0.3 4/2) 100))
     (frame :name :thermistor-mask :cross-section (m/circle (/ 3.4 2) 100))
     (translate :x 2 :to [:thermistor-mask])
     (translate :x -2 :to [:feed-mask])
     (forward :length 1.2)
     (set :cross-section (m/offset outer-shape -0.8) :to [:feed-mask])
     (translate :x 2 :to [:feed-mask])
     (forward :length 9))))

(u/in->mm 1/8)

(def build-plate-support)

(comment

  (export-models *ns* "glb")

  (export-models *ns* "f3z")
  (export-models *ns* "stl")

  )
