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
            intersection iso-hull mirror]]))

(def rod-mount-offset 21)
(def rod-mount-margin (- 65/2 rod-mount-offset 3))
(def carriage-spacer-length 2.5)
(def outer-mount-length (- rod-mount-margin carriage-spacer-length))

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
    (-> (m/union
         (m/cross-section pts)
         (-> (m/square 100 0.6 false)
             (m/translate [0 (- (* 1 0.6))])))
        (m/translate [-50 0]))))

(def carriage-mount
  (extrude
   (result :name :carriage-mount
           :expr (union (difference :fisheye-mounts :belt-mask)
                        (difference :carriage-mount-body :bolt-masks :belt-mask :fisheye-mount-mask-composite :endstop-screw)))

   (frame :cross-section (m/square 65 65 true)
          :name :carriage-mount-body)

   (branch
    :from :carriage-mount-body
    :with []
    (frame :name :origin)
    (translate :z 13)
    (segment (:forms fisheye-mounts)))

   (branch
    :from :carriage-mount-body
    :with []
    (frame :name :endstop-screw
           :cross-section (m/circle (/ 1.8 2) 20))
    (translate :y (- (- 65/2 19.8)) :z 4)
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
                                  (-> (m/square 18 8 true)
                                      (m/translate [0 (- 2.7)]))
                                  (-> (m/square 100 4 true)
                                      (m/translate [0 (- (- (/ 13.5 2) 0.7))])
                                      (m/rotate T))) )

   (set :cross-section (m/union (m/square 45 25 true)
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
