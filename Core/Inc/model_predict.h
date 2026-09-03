/*
 * model_predict.h
 *
 * C-callable wrapper around the micromlgen-exported RandomForest
 * regressor (distance_regressor.h), so plain-C files like rf_task.c
 * can call it. The model itself is C++ (a class in a namespace) -
 * this header/wrapper pair is what bridges the two.
 */

#ifndef INC_MODEL_PREDICT_H_
#define INC_MODEL_PREDICT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* features[5] must be in this exact order: [mean, std, min, max, median],
 * matching FEATURE_COLS from train_model.py. Returns predicted distance
 * in meters. */
float predict_distance(const float features[5]);

#ifdef __cplusplus
}
#endif

#endif /* INC_MODEL_PREDICT_H_ */
