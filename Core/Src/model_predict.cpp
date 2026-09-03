/*
 * model_predict.cpp
 *
 *  Created on: Sep 3, 2026
 *      Author: Tarun Kumar M
 */
/*
 * model_predict.cpp
 *
 * Instantiates the micromlgen-exported RandomForestRegressor and
 * exposes a single plain-C-callable predict function. This file is
 * C++ (note the .cpp extension) - that's required because the
 * exported model itself is a C++ class inside a namespace.
 *
 * REQUIRES: your trained distance_regressor.h (exported by
 * train_model.py via micromlgen) must be placed in Core/Inc/ with
 * exactly that filename before this file will compile.
 */

#include "model_predict.h"
#include "distance_regressor.h"

/* One static instance, reused across every prediction - avoids
 * re-constructing the tree structure on every call. */
static Eloquent::ML::Port::RandomForestRegressor clf;

extern "C" float predict_distance(const float features[5]) {
    /* micromlgen's predict() takes a plain float*, not const - copy
     * into a local mutable array rather than casting away const on
     * the caller's data. */
    float f[5];
    for (int i = 0; i < 5; i++) {
        f[i] = features[i];
    }
    return clf.predict(f);
}
