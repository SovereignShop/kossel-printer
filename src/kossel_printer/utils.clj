(ns kossel-printer.utils
  (:require
   [kossel-printer.math :refer [pi TT T T|2 T|3 T|4 T|5 T|6]]
   [sicmutils.env :as e]
   [sicmutils.calculus.derivative :as cd]
   [plexus.transforms :as tf]))

(defn circle-pts [r res]
  (let [pts (for [i (range res)]
              (-> tf/identity-tf
                  (tf/rotate :z (* i (/ TT res)))
                  (tf/go-forward r :x)
                  (tf/translation-vector)
                  (subvec 0 2)))]
    pts))

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