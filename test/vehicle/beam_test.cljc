(ns vehicle.beam-test
  "Ported 1:1 from kami-vehicle's `src/beam.rs` `#[cfg(test)] mod tests`."
  (:require [clojure.test :refer [deftest is]]
            [vehicle.beam :as beam]))

(deftest normal-beam-relaxed-has-zero-force
  (let [b (beam/new-beam 0 0 1 1.0 1000.0 10.0)]
    (is (< (Math/abs (beam/force-scalar b 1.0 0.0 0.0)) 1e-3))))

(deftest stretched-beam-pulls-nodes-together
  (let [b (beam/new-beam 0 0 1 1.0 1000.0 10.0)]
    (is (< (Math/abs (+ (beam/force-scalar b 1.10 0.0 0.0) 100.0)) 1e-2))))

(deftest bounded-beam-idle-inside-window
  (let [b (-> (beam/new-beam 0 0 1 1.0 1000.0 10.0)
              (beam/with-type (beam/beam-type-bounded 0.8 1.2)))]
    (is (= 0.0 (beam/force-scalar b 1.05 0.0 0.0)))
    (is (< (beam/force-scalar b 1.30 0.0 0.0) 0.0))))

(deftest hydro-beam-extends-with-control
  (let [b (-> (beam/new-beam 0 0 1 1.0 1000.0 10.0)
              (beam/with-type (beam/beam-type-hydro 0.20 1.0)))]
    (is (< (Math/abs (beam/force-scalar b 1.20 0.0 0.0)) 1e-3))))

(deftest pressured-beam-at-reference-pressure-is-relaxed
  ;; At pressure == reference-pressure (a healthy, un-punctured tire), the
  ;; live rest length must equal the beam's effective-length exactly --
  ;; no shrink, no growth (the multiplier reduces to 1.0).
  (let [b (-> (beam/new-beam 0 0 1 0.30 1000.0 10.0)
              (beam/with-type (beam/beam-type-pressured 0.05 2.4)))]
    (is (= 0.30 (beam/live-rest-length b 2.4)))
    (is (< (Math/abs (beam/force-scalar b 0.30 0.0 2.4)) 1e-3))))

(deftest pressured-beam-below-reference-pressure-shrinks
  (let [b (-> (beam/new-beam 0 0 1 0.30 1000.0 10.0)
              (beam/with-type (beam/beam-type-pressured 0.05 2.4)))]
    (is (< (beam/live-rest-length b 1.0) 0.30))))

(deftest beam-yields-past-deform-limit
  (let [b (beam/new-beam 0 0 1 1.0 1000.0 10.0)
        [b' broke?] (beam/update-plastic b 1.20)]
    (is (not broke?))
    (is (> (:effective-length b') 1.0))))

(deftest beam-breaks-past-break-limit
  (let [b (beam/new-beam 0 0 1 1.0 1000.0 10.0)
        [b' broke?] (beam/update-plastic b 1.50)]
    (is broke?)
    (is (:broken b'))))
