//
// Created by rprzybyla on 8/24/2021.
//

#ifndef SHASTA_GPT_ICU_SHASTA_ALGO_STRUCTS_H
#define SHASTA_GPT_ICU_SHASTA_ALGO_STRUCTS_H

#include "icu_rangefinder_interface.h"

#define ICU_SHASTA_ALGO_VERSION_X 1 //this should be updated whenever ICU_ALGO_SHASTA_CONFIG or ICU_ALGO_SHASTA_OUTPUT change format
#define ICU_SHASTA_ALGO_VERSION_Y 1
#define ICU_SHASTA_ALGO_VERSION_Z 0
#define ICU_SHASTA_ALGO_VERSION_W 0
#define ICU_SHASTA_ALGO_ID 1

typedef InvnAlgoRangeFinderConfig ICU_ALGO_SHASTA_CONFIG;
typedef InvnAlgoRangeFinderOutput ICU_ALGO_SHASTA_OUTPUT;
typedef InvnAlgoRangeFinder ICU_ALGO_SHASTA_STATE;
#endif //SHASTA_GPT_ICU_SHASTA_ALGO_STRUCTS_H
