(ns kossel-printer.utils
  (:require
   [kossel-printer.math :refer [pi TT T T|2 T|3 T|4 T|5 T|6]]
   [sicmutils.env :as e]
   [sicmutils.calculus.derivative :as cd]
   [plexus.transforms :as tf]
   [plexus.triangles :as triangles]
   [plexus.core :as p]
   [clj-manifold3d.core :as m]))

(defn in->mm [inches]
  (* inches 25.4))

(defn ft->mm [ft]
  (* ft 12 25.4))

(defn mm->in [mm]
  (/ mm 25.4))

(defn circle-pts [r res]
  (let [pts (for [i (range res)]
              (-> tf/identity-tf
                  (tf/rotate :z (* i (/ (* 2 pi) res)))
                  (tf/go-forward r :x)
                  (tf/translation-vector)
                  (subvec 0 2)))]
    pts))

(defn circumference [r]
  (* 2 Math/PI r))

(defn ovol
  ([rx ry]
   (ovol rx ry 100))
  ([rx ry n-steps]
   (m/cross-section
    (for [x (range n-steps)]
      (let [d (* x (/ (* 2 Math/PI) n-steps))]
        [(* rx (Math/cos d))
         (* ry (Math/sin d))])))))

(defn cos-wave-points
  ([width height]
   (cos-wave-points width height 1))
  ([width height n-waves]
   (cos-wave-points width height n-waves 0))
  ([width height n-waves offset]
   (let [steps 20
         step-size (+ 0.02 (/ (* 2 pi) steps))
         length (* n-waves 2 pi)
         dcos (cd/D e/cos)
         f (fn [x]
             (let [dv [(dcos x) -1]
                   ;; normal perp vector to tangent line, scaled by offset
                   [dx dy] (if (or (pos? offset) (neg? offset))
                             (e/* (- offset) (e// dv (e/magnitude dv)))
                             [0 0])]
               [(+ (- dx) (* (/ width 2) (/ x pi)))
                (+ (/ height 2) (- dy) (* (/ height 2) (Math/cos x)))]))]
     (conj (vec
            (for [x (range (- pi) (- length pi) step-size)]
              (f x)))
           (f (- length pi))))))

(defn arc-segment [side-length curve-radius props]
  (let [angle (triangles/abc->A side-length curve-radius curve-radius)
        r (- (/ Math/PI 2) (/ (- Math/PI angle) 2))]
    [(p/rotate :y r)
     (p/left :angle angle :curve-radius curve-radius :props props)
     (p/rotate :y r)]))

(defn triangle-centroid [l]
  (let [cx (/ l 2)
        cy (/ (* (Math/sqrt 3) l) 6)]
    {:x cx, :y cy}))

(defn curved-triangle-points [side-length curve-radius]
  (p/points
   :axes [:x :y]
   (p/frame :name :origin :cs 20)
   (p/rotate :x (- (/ Math/PI 2)))
   (let [{:keys [x y]} (triangle-centroid side-length)]
     (p/translate :x x :y y))
   (p/rotate :y (- (/ Math/PI 3)))
   (for [_ (range 3)]
     [(arc-segment side-length curve-radius {})
      (p/rotate :y (- (* 2 (/ Math/PI 3))))])))

(defn curved-triangle [side-length curve-radius]
  (m/cross-section (curved-triangle-points side-length curve-radius)))

(defn thread [section pitch radius steps-per-revolution n-revolutions]
  (m/loft (m/union (for [i (range n-revolutions)]
                     (-> section
                         (m/translate [0 (- (* i pitch))]))))
          (for [step (range (inc steps-per-revolution))]
            (-> (m/frame 1)
                (m/rotate [0 0 (* (/ (* 2 Math/PI) steps-per-revolution) step)])
                (m/translate [radius 0 (* step (/ pitch steps-per-revolution))])
                (m/rotate [(- (/ Math/PI 2)) 0 0])))))

(comment

  (m/union
   (m/cylinder (* 6 15) 40 40 100)
   (thread (m/circle 3 4) 6 40 100 15))

  )
