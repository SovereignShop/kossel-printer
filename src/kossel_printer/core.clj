(ns kossel-printer.core
  (:require
   [plexus.utils :as pu]
   [plexus.transforms :as tf]
   [clj-manifold3d.core :as m]
   [kossel-printer.utils :as u]
   [kossel-printer.math :refer [pi pi|2 pi|3 pi|4 pi|5 pi|6 two-pi
                                TT T T|2 T|3 T|4 T|5 T|6
                                sin asin cos acos sqrt sqr atan tan]]
   [plexus.core :as paths
    :refer [model forward hull left right up down roll backward translate spin
            slice set segment pattern transform branch offset minkowski
            rotate frame mask save-transform add-ns extrude to points loft
            result difference subtract intersect join union show-coordinate-frame
            intersection iso-hull mirror lookup-transform add-ns]]))

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

(comment
  (:carriage-mask (:frames printer-model))

  )

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
    #_(-> (m/union
         (m/cross-section pts)
         (-> (m/square 100 0.9 false)
             (m/translate [0 (- (* 1 0.9))])))
        (m/translate [-50 0]))))

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
   #_(for [_ (range 4)]
     [(offset :delta 0.1 :to [:belt-mask] :simplify 0.001)
      (forward :length 0.2)])
   (forward :length 8 :to [:belt-mask])))

#_(m/hull (m/cylinder 0.1 10)
        (-> (m/cube 4 4 0.1 true)
            (m/translate [0 0 10])))

(let [m (m/union (-> belt-mask-polygon
                     (m/translate [0 (- (- (/ 13.5 2) 0.7))]))
                 (-> belt-mask-polygon
                     (m/translate [0 (- (- (/ 13.5 2) 6))])
                     (m/rotate T))
                 (-> (m/square 19 8 true)
                     (m/translate [0 (- 2.7)]))
                 (-> (m/square 100 4 true)
                     (m/translate [0 (- (- (/ 13.5 2) 0.7))])
                     (m/rotate T)))
      m2 (m/simplify (m/offset m -0.2) 0.0001)]
  m
  #_(reduce + (map (comp count seq) (m/to-polygons m)))
  #_(m/loft [m m2]
            [(m/frame 1) (-> (m/frame 1) (m/translate [0 0 1]))]))

(def split-carriage-mount
  (let [[left right] (m/split-by-plane (:carriage-mount (:frames carriage-mount))
                                       [0 1 0] 8.05)]
    (m/union (m/translate left [0 1 0])
             right)))
