(ns kossel-printer.core
  (:require
   [plexus.utils :as pu]
   [plexus.transforms :as tf]
   [clj-manifold3d.core :as m]
   [kossel-printer.utils :as u]
   [kossel-printer.math :refer [pi pi|2 pi|3 pi|4 pi|5 pi|6 two-pi
                                TT T T|2 T|3 T|4 T|5 T|6
                                sin asin cos acos sqrt sqr atan tan]]
   [kossel-printer.offset-polygon :refer [offset-polygon]]
   [plexus.core :as paths
    :refer [model forward hull left right up down roll backward translate spin
            slice set segment pattern transform branch offset minkowski
            rotate frame mask save-transform add-ns extrude to points loft
            result difference subtract intersect join union show-coordinate-frame
            intersection iso-hull mirror lookup-transform add-ns get-frame]]))

(def rod-mount-offset 21)
(def rod-mount-margin (- 65/2 rod-mount-offset 3))

(def printer-height (u/in->mm 48))
(def heated-bed-radius 310/2)
(def build-plate-radius 310/2)
(def carriage-spacer-length 2.5)
(def outer-mount-length (- rod-mount-margin carriage-spacer-length))

(def printer-offset 350)

(def panel-width (u/in->mm 22))
(def panel-height (u/in->mm 48))
(def panel-thickness 4)

;; Offset from edge of vertical aluminum extrusion that faces the build plate.
;; Offset is towards the center of the plate.
(def stepper-mount-frame-offset 38)
(def belt-shaft-mask-offset 19)
(def belt-shaft-width 30)

(def printer-model
  (extrude
   (result :name :printer-model :expr :body)
   (frame :cross-section (m/square 20.4 20.4 true) :name :body)
   (translate :x (- 457/2 50) :y 1727/2)
   (for [x (range 1)]
     (branch
      :from :body
      (add-ns :namespace (keyword (str "k" x)))
      (translate :x (if (even? x) 0 100) :y (- (* x printer-offset) (* 2 printer-offset)))
      (rotate :z (if (even? x) 0 Math/PI))
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
        :from :body
        :with []
        (frame :cross-section (m/circle (+ 60 heated-bed-radius))
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
           :from :body
           :with []
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
           :from :body
           :with []
           (frame :cross-section (-> (m/square 44 68 true)
                                     (m/offset (- 5) :square)
                                     (m/offset 5 :round)
                                     (m/simplify 0.1))
                  :name :carriage-mask)

           (translate :x -11.8)
           (translate :z 62.5)
           (forward :length (- printer-height 62.5)))
          (forward :length printer-height))

         (branch
          :from :body
          :with []
          (frame :name :side-rod-mask
                 :cross-section (m/square 20.4 61 true))
          (rotate :z pi|3)
          (translate :x 145 :z 30)
          (rotate :x pi|2)
          (forward :length 400 :center true))

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

(def build-plate-mask
  (extrude
   (frame :name :build-plate-mask
          :cross-section (m/circle build-plate-radius 300))

   (transform :replace (lookup-transform printer-model :k0/bed))
   (forward :length 100 :center true)))

(def build-plate-support
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
                                        (m/translate [6 0]))))
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

(def psu-mount
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


(def extruder-mount
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

(def board-box
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

   (segment
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
       (forward :length 3))))

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
      (set :cross-section (m/square 4 140 true))
      (translate :x -1/2)
      (save-transform :frame :board-mount-body :name ::mount-frame)
      (hull
       (forward :length 1)
       (translate :z 4)
       (set :cross-section (m/square 2 140 true))
       (translate :x 1)
       (forward :length 0.01))
      #_(show-coordinate-frame)))))

(let [y1 (.y (.getColumn (lookup-transform board-box ::start-frame) 3))
      y2 (.y (.getColumn (lookup-transform board-box ::mount-frame) 3))]
  (def box-lid-width (+ 4 (* 2 (- y2 y1)))))

(def board-box-lid
  (extrude
   (result :name :board-box-lid
           :expr (union :board-box-lid-body :plate))

   (frame :name :board-box-lid-body
          :cross-section (m/square 1.6 140 true))

   (for [i (range 2)]
     (branch
      :from :board-box-lid-body
      (rotate :x (+ (* i pi) (- pi|2)))
      (forward :length (/ box-lid-width 2))
      (left :angle pi|2 :curve-radius 0.8)
      (forward :length 5.5)
      (translate :x -1/2)
      (set :cross-section (m/square 2.6 140 true))
      (loft
       (forward :length 1)
       (translate :z 4 :x 0.5)
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

(def power-switch-box
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

     (transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))

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
         (forward :length 5))))

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
         :with []
         (frame :cross-section (m/square 11 7 true) :name :horizontal-bolt-holes)
         (for [z [10 55]
               y [(if (= i 0) 2.5 -2.5) -217.5 217.5]]
           (branch
            :from :horizontal-bolt-holes
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

(def tower-support-top
  (m/difference (get-frame tower-support-top-impl :tower-support-top)
                (get-frame printer-model :body)))

(def tower-support-top-bolt-housing
  (get-frame tower-support-top-impl :bolt-housing))

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
    (m/square 100 2.15 true)
    ))

(def carriage-mount
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

   (set :cross-section (m/union (m/square 45 25 true)
                                (m/hull (-> (m/square 45 3 true)
                                            (m/translate [0 2.75]))
                                        (-> (m/square 3 0.2 true)
                                            (m/translate [(+ 45/2 3) 2.75])))
                                (m/translate (m/hull (m/square 45 5.8 true)
                                                     (-> (m/square 3 0.2 true)
                                                         (m/translate [(+ 45/2 3) 0])))
                                             [0 -9.65])
                                (m/translate (m/hull (m/square 45 5.8 true)
                                                     (-> (m/square 3 0.2 true)
                                                         (m/translate [(- (+ 45/2 3)) 0])))
                                             [0 -9.65])

                                (m/hull (-> (m/square 45 5 true)
                                            (m/translate [0 -2]))
                                        (-> (m/square 3 0.2 true)
                                            (m/translate [(+ 45/2 3) -2])))
                                (m/hull (-> (m/square 45 5 true)
                                            (m/translate [0 -2]))
                                        (-> (m/square 3 0.2 true)
                                            (m/translate [(- (+ 45/2 3)) -2]))))
        :to [:carriage-mount-body])
   (forward :length 6)
   (forward :length 8 :to [:belt-mask])))

(def split-carriage-mount
  (let [[left right] (m/split-by-plane (:carriage-mount (:frames carriage-mount))
                                       [0 1 0] 8.05)]
    (m/union (m/translate left [0 1 0])
             right)))


(def tower-support-bolt-holes
  (extrude
   (result :name :tower-support-bolt-holes :expr :side-rod-bolt-point)
   (frame :name :side-rod-bolt-point :cross-section (m/square 7 11 true))
   (for [tf-name [:k0.e0.r0/side-rod :k0.e0.r1/side-rod]]
     (branch
      :from :side-rod-bolt-point
      (transform :replace (lookup-transform printer-model tf-name))
      (translate :z 10)
      (translate :y (/ 360 2))
      (rotate :y pi|2)
      (for [x [0 -25]]
        (branch
         :from :side-rod-bolt-point
         (translate :y x)
         (forward :length 13)
         (set :cross-section (m/circle 14/2))
         (forward :length 50)))))))

(def tower-support
  (extrude
   (result :name :tower-support
           :expr (difference
                  :tower-support-body
                  :tower-support-mask
                  :stepper-bolt-mask
                  :stepper-shaft-hole
                  :tower-support-bolt-holes
                  :vertical-rod-bolt-holes))

   (frame :cross-section (m/square 10 10 true) :name :tower-support-body)
   (branch :from :tower-support-body :with [] (:forms tower-support-bolt-holes))
   (transform :replace (lookup-transform printer-model :k0.e1/vertical-rod))
   (branch
    :from :tower-support-body
    :with []
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
    :from :tower-support-body
    :with []
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
             :cross-section (m/union (for [x [15 -15]
                                           y [15 -15]]
                                       (-> (m/circle 1.6)
                                           (m/translate [x y])))))
      (frame :name :stepper-shaft-hole
             :cross-section (m/circle 11.2))
      (rotate :y pi)
      (forward :length 4)
      (offset :delta 6)
      (forward :length 8 :to [:stepper-shaft-hole])
      (forward :length 200 :to [:stepper-bolt-mask]))
     (forward :length 200)))

   (branch
    :from :tower-support-body
    (frame :name :tower-support-mask :cross-section (m/square belt-shaft-width belt-shaft-width true))
    (translate :x (- (+ 10 belt-shaft-mask-offset)))
    (forward :length 200))

   (hull
    (segment
     (for [sign [-1 1]
           [height length] [[5 62.5]
                            [60 62.5]
                            [190 10]]]
       [(transform :replace (lookup-transform printer-model :k0.e1/vertical-rod))
        (translate :z height :x 85)
        (rotate :z (* sign pi|6))
        (rotate :y (- pi|2))
        (translate :z 80)
        (forward :length length)
        (rotate :x (* sign (- (+ pi|6 pi|3))))
        (forward :length 10)])))))

(def base-tower-support
  (m/difference (get-frame tower-support :tower-support)
                (get-frame printer-model :carriage-mask)
                (get-frame printer-model :side-rod-mask)
                (get-frame printer-model :body)
                (get-frame printer-model :heated-bed-mask)))

(def build-plate-support-2
  (extrude

   (result :name :build-plate-support
           :expr (difference :build-plate-support-body
                             :bolt-mask
                             :build-plate-mask))

   (frame :cross-section (m/square 6 165 true)
          :name :build-plate-support-body)

   (branch
    :from :build-plate-support-body
    :with []
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

(def carriage-heat-shield-mount
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

(def middle-heat-shield-mount
  (extrude
   (result :name :middle-heat-shield-mount
           :expr (difference :mount-body :mount-mask))

   (frame :name :mount-body
          :cross-section (m/square 4 20 true))

   (transform :replace (lookup-transform printer-model :k0.e0/vertical-rod))
   (translate :z 400 :x 12.2)

   (branch
    :from :mount-body
    :with []
    (frame :name :mount-mask
           :cross-section (m/circle 8))
    (translate :x 2)
    (rotate :y (- pi|2))
    (forward :length 2)
    (set :cross-section (m/circle 2))
    (forward :length 5))

   (for [i (range 2)]
     (branch
      :from :mount-body
      (rotate :x (+ pi|2 (* i pi)))
      (forward :length 10)
      (right :angle pi|4 :curve-radius 2)
      (left :angle pi|4 :curve-radius 2)
      (forward :length 21)
      (hull
       (left :angle pi|2 :curve-radius 10)
       (forward :length (+ 20 22))
       (translate :x 15)
       (rotate :x pi)
       (forward :length 1))))))
