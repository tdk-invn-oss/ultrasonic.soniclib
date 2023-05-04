//
// Created by rprzybyla on 9/8/2021.
//

#ifndef SHASTA_GPT_SHASTA_GPT_INTERFACE_H
#define SHASTA_GPT_SHASTA_GPT_INTERFACE_H

#include "icu_rangefinder_interface.h"
#include "icu_algo_info.h"

//This defines the algo structure for shasta_gpt.
typedef struct shasta_gpt{
    InvnAlgoRangeFinderOutput algo_out; // will be pass as a pointer to algo process
    volatile InvnAlgoRangeFinderConfig algo_cfg; // will be pass as a pointer to algo init
    ICU_ALGO_SHASTA_INFO algo_info;
} shasta_gpt_t;


#endif //SHASTA_GPT_SHASTA_GPT_INTERFACE_H
