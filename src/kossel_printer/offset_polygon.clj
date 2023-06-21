(ns kossel-printer.offset-polygon)

(defn vec-add
  [v1 v2]
  (with-meta
    [(+ (first v1) (first v2)) (+ (second v1) (second v2))]
    (meta v1)))

(defn vec-sub
  [v1 v2]
  (with-meta
    [(- (first v1) (first v2)) (- (second v1) (second v2))]
    (meta v1)))

(defn vec-scale
  [v scalar]
  (with-meta
    [(* (first v) scalar) (* (second v) scalar)]
    (meta v)))

(defn vec-length
  [v]
  (Math/sqrt (+ (Math/pow (first v) 2) (Math/pow (second v) 2))))

(defn vec-normalize [v]
  (let [magnitude (Math/sqrt (reduce + (map * v v)))]
    (with-meta (mapv #(/ % magnitude) v)
      (meta v))))

(defn vec-dot
  [v1 v2]
  (+ (* (first v1) (first v2)) (* (second v1) (second v2))))

(defn angle-between-vectors [v1 v2]
  (let [dot-product (vec-dot v1 v2)
        magnitudes (* (vec-length v1) (vec-length v2))]
    (Math/acos (min 1 (max -1 (/ dot-product magnitudes))))))

(defn cross-product-z
  [v1 v2]
  (- (* (nth v1 0) (nth v2 1)) (* (nth v1 1) (nth v2 0))))

(def epsilon 1e-6)

(defn near-zero? [x]
  (< (Math/abs x) epsilon))

(defn collinear?
  [p1 p2 p3]
  (near-zero? (cross-product-z (vec-sub p2 p1) (vec-sub p3 p1))))

(defn bisect-point-extended
  [p1 p2 p3 offset]
  (if (collinear? p1 p2 p3)
    (let [v1 (if (= p1 p2)
               (vec-normalize (vec-sub p3 p2))
               (vec-normalize (vec-sub p2 p1)))
          v2 (if (= p2 p3)
               (vec-normalize (vec-sub p2 p1))
               (vec-normalize (vec-sub p3 p2)))
          perp [(- (second v1)) (first v1)]
          sign (if (> (vec-dot perp v2) epsilon) 1 -1)
          offset-point (vec-add p2 (vec-scale perp (* sign offset)))]
      (with-meta offset-point (meta p2)))
    (let [v1 (vec-normalize (vec-sub p2 p1))
          v2 (vec-normalize (vec-sub p2 p3))
          bisector (vec-normalize (vec-add v1 v2))
          angle (angle-between-vectors v1 v2)
          bisector-len (/ offset (Math/sin (/ angle 2)))
          sign (if (> (cross-product-z v1 v2) 0) -1 1)
          extended-point (with-meta (vec-add p2 (vec-scale bisector (* sign bisector-len)))
                           (meta p2))]
      extended-point)))

(defn offset-polygon
  [points offset & {:keys [wrap] :or {wrap true}}]
  (let [n (count points)
        offset-points (map-indexed
                       (fn [i _]
                         (let [prev (if (= i 0)
                                      (if wrap (dec n) i)
                                      (dec i))
                               next (if (= i (dec n))
                                      (if wrap 0 i)
                                      (inc i))]
                           (bisect-point-extended
                            (nth points prev) (nth points i) (nth points next) offset)))
                       points)]
    (vec offset-points)))

(defn distance [p1 p2]
  (Math/sqrt (+ (Math/pow (- (p2 0) (p1 0)) 2) (Math/pow (- (p2 1) (p1 1)) 2))))

(defn lerp [p1 p2 t]
  [(+ (* (- (p2 0) (p1 0)) t) (p1 0)) (+ (* (- (p2 1) (p1 1)) t) (p1 1))])

(defn add-points-to-edge [p1 p2 num-points]
  (let [interval (/ 1.0 (inc num-points))]
    (mapv #(with-meta (lerp p1 p2 (* % interval)) (meta p2)) (range 0 (inc num-points)))))

(defn add-points-to-polygon [polygon num-points]
  (if (< (count polygon) 2)
    (throw (IllegalArgumentException. "Polygon must have at least 2 vertices."))
    (let [xf (mapcat (fn [[p1 p2]] (cons p1 (add-points-to-edge p1 p2 num-points))))
          edges (partition 2 1 (conj (vec polygon) (first polygon)))]
      (sequence xf edges))))

(defn normal [p1 p2]
  (vec-normalize [(- (p1 1) (p2 1)) (- (p2 0) (p1 0))]))

(defn normal [p1 p2]
  (vec-normalize [(- (p2 1) (p1 1)) (- (p1 0) (p2 0))]))

(defn transform-polygon [polygon transform-fn]
  (if (< (count polygon) 2)
    (throw (IllegalArgumentException. "Polygon must have at least 2 vertices."))
    (let [edges (partition 2 1 (conj (vec polygon) (first polygon)))
          transformed-points (map-indexed (fn [idx [p1 p2]] (transform-fn idx p1 (normal p1 p2))) edges)]
      (vec transformed-points))))

(defn add-points-to-polygon [polygon num-points]
  (if (< (count polygon) 2)
    (throw (IllegalArgumentException. "Polygon must have at least 2 vertices."))
    (let [xf (map (fn [[p1 p2]] (cons p1 (add-points-to-edge p1 p2 num-points)))
                  (partition 2 1 (conj (vec polygon) (first polygon))))]
      (->> xf
           (reduce into [])
           vec))))

(defn distance [p1 p2]
  (Math/sqrt (reduce + (map #(* (- %2 %1) (- %2 %1)) p1 p2))))

(defn line-intersection [p1 p2 p3 p4]
  (let [denom (- (* (- (p2 1) (p1 1)) (- (p4 0) (p3 0))) (* (- (p2 0) (p1 0)) (- (p4 1) (p3 1))))
        ua-num (* (- (p4 1) (p3 1)) (- (p1 0) (p3 0)) (- (p4 0) (p3 0)) (- (p1 1) (p3 1)))
        ub-num (* (- (p2 1) (p1 1)) (- (p1 0) (p3 0)) (- (p2 0) (p1 0)) (- (p1 1) (p3 1)))]

    (if (zero? denom)
      nil
      (let [ua (/ ua-num denom)
            ub (/ ub-num denom)]
        (if (and (<= 0 ua 1) (<= 0 ub 1))
          [(+ (* ua (- (p2 0) (p1 0))) (p1 0)) (+ (* ua (- (p2 1) (p1 1))) (p1 1))]
          nil)))))

(defn find-intersection [polygon start direction]
  (let [edges (partition 2 1 (conj (vec polygon) (first polygon)))
        intersections (keep (fn [[p1 p2]] (line-intersection p1 p2 start (mapv + start direction))) edges)]
    (first (sort-by #(distance start %) intersections))))
