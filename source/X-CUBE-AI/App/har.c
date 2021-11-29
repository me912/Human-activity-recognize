/**
  ******************************************************************************
  * @file    har.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Nov 19 11:55:03 2021
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


#include "har.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"




#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_har
 
#undef AI_HAR_MODEL_SIGNATURE
#define AI_HAR_MODEL_SIGNATURE     "0b685d4e89a2ce5ad6cb0bf8fa253243"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Fri Nov 19 11:55:03 2021"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_HAR_N_BATCHES
#define AI_HAR_N_BATCHES         (1)




/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_0_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 1152, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3200, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  average_pooling1d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 625, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 25, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_nl_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_nl_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 6, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_kernel_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 900, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_recurrent_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2500, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_peephole_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 75, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 100, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_kernel_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2500, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_recurrent_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2500, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_peephole_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 75, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 100, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 750, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 600, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  dense_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 20, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 120, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  dense_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  lstm_4_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 175, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  lstm_5_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 175, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  input_0_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 9, 1, 128), AI_STRIDE_INIT(4, 4, 4, 36, 36),
  1, &input_0_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 25, 1, 128), AI_STRIDE_INIT(4, 4, 4, 100, 100),
  1, &lstm_4_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  average_pooling1d_4_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 25, 1, 25), AI_STRIDE_INIT(4, 4, 4, 100, 100),
  1, &average_pooling1d_4_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 25, 1, 1), AI_STRIDE_INIT(4, 4, 4, 100, 100),
  1, &lstm_5_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 1), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &dense_3_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_nl_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 1), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &dense_3_nl_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_nl_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_nl_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_5_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_nl_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_5_nl_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_kernel, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 9, 1, 1, 100), AI_STRIDE_INIT(4, 4, 36, 36, 36),
  1, &lstm_4_kernel_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_recurrent, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 25, 1, 1, 100), AI_STRIDE_INIT(4, 4, 100, 100, 100),
  1, &lstm_4_recurrent_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_peephole, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 75, 1, 1), AI_STRIDE_INIT(4, 4, 4, 300, 300),
  1, &lstm_4_peephole_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_bias, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 100, 1, 1), AI_STRIDE_INIT(4, 4, 4, 400, 400),
  1, &lstm_4_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_kernel, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 25, 1, 1, 100), AI_STRIDE_INIT(4, 4, 100, 100, 100),
  1, &lstm_5_kernel_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_recurrent, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 25, 1, 1, 100), AI_STRIDE_INIT(4, 4, 100, 100, 100),
  1, &lstm_5_recurrent_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_peephole, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 75, 1, 1), AI_STRIDE_INIT(4, 4, 4, 300, 300),
  1, &lstm_5_peephole_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_bias, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 100, 1, 1), AI_STRIDE_INIT(4, 4, 4, 400, 400),
  1, &lstm_5_bias_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_weights, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 25, 30, 1, 1), AI_STRIDE_INIT(4, 4, 100, 3000, 3000),
  1, &dense_3_weights_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_bias, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 1), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &dense_3_bias_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_weights, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 30, 20, 1, 1), AI_STRIDE_INIT(4, 4, 120, 2400, 2400),
  1, &dense_4_weights_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  dense_4_bias, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 1, 1), AI_STRIDE_INIT(4, 4, 4, 80, 80),
  1, &dense_4_bias_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_weights, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 20, 6, 1, 1), AI_STRIDE_INIT(4, 4, 80, 480, 480),
  1, &dense_5_weights_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  dense_5_bias, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &dense_5_bias_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  lstm_4_scratch0, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 175, 1, 1), AI_STRIDE_INIT(4, 4, 4, 700, 700),
  1, &lstm_4_scratch0_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  lstm_5_scratch0, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 175, 1, 1), AI_STRIDE_INIT(4, 4, 4, 700, 700),
  1, &lstm_5_scratch0_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_5_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_5_nl_layer, 7,
  NL_TYPE, 0x0, NULL,
  nl, forward_sm,
  &dense_5_nl_chain,
  NULL, &dense_5_nl_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_5_weights, &dense_5_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_5_layer, 7,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_5_chain,
  NULL, &dense_5_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_4_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_4_nl_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_4_nl_chain,
  NULL, &dense_5_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_nl_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_4_weights, &dense_4_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_4_layer, 6,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_4_chain,
  NULL, &dense_4_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_nl_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_nl_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_nl_layer, 4,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_3_nl_chain,
  NULL, &dense_4_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_3_weights, &dense_3_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_layer, 4,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_3_chain,
  NULL, &dense_3_nl_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  lstm_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &average_pooling1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 7, &lstm_5_kernel, &lstm_5_recurrent, &lstm_5_peephole, &lstm_5_bias, NULL, NULL, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_5_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  lstm_5_layer, 2,
  LSTM_TYPE, 0x0, NULL,
  lstm, forward_lstm,
  &lstm_5_chain,
  NULL, &dense_3_layer, AI_STATIC, 
  .n_units = 25, 
  .activation_nl = nl_func_tanh_array_f32, 
  .go_backwards = false, 
  .reverse_seq = false, 
  .out_nl = nl_func_tanh_array_f32, 
  .recurrent_nl = nl_func_sigmoid_array_f32, 
  .cell_clip = 3e+38, 
  .state = AI_HANDLE_PTR(NULL), 
  .init = AI_LAYER_FUNC(NULL), 
  .destroy = AI_LAYER_FUNC(NULL), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  average_pooling1d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &average_pooling1d_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  average_pooling1d_4_layer, 1,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &average_pooling1d_4_chain,
  NULL, &lstm_5_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(1, 5), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 5), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  lstm_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 7, &lstm_4_kernel, &lstm_4_recurrent, &lstm_4_peephole, &lstm_4_bias, NULL, NULL, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &lstm_4_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  lstm_4_layer, 0,
  LSTM_TYPE, 0x0, NULL,
  lstm, forward_lstm,
  &lstm_4_chain,
  NULL, &average_pooling1d_4_layer, AI_STATIC, 
  .n_units = 25, 
  .activation_nl = nl_func_tanh_array_f32, 
  .go_backwards = false, 
  .reverse_seq = false, 
  .out_nl = nl_func_tanh_array_f32, 
  .recurrent_nl = nl_func_sigmoid_array_f32, 
  .cell_clip = 3e+38, 
  .state = AI_HANDLE_PTR(NULL), 
  .init = AI_LAYER_FUNC(NULL), 
  .destroy = AI_LAYER_FUNC(NULL), 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 41104, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 13500, 1,
                     NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_HAR_IN_NUM, &input_0_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_HAR_OUT_NUM, &dense_5_nl_output),
  &lstm_4_layer, 0, NULL)



AI_DECLARE_STATIC
ai_bool har_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)
  AI_UNUSED(net_ctx)

  ai_ptr activations_map[1] = AI_C_ARRAY_INIT;

  if (ai_platform_get_activations_map(activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    input_0_output_array.data = AI_PTR(NULL);
    input_0_output_array.data_start = AI_PTR(NULL);
    lstm_4_output_array.data = AI_PTR(activations_map[0] + 700);
    lstm_4_output_array.data_start = AI_PTR(activations_map[0] + 700);
    average_pooling1d_4_output_array.data = AI_PTR(activations_map[0] + 700);
    average_pooling1d_4_output_array.data_start = AI_PTR(activations_map[0] + 700);
    lstm_5_output_array.data = AI_PTR(activations_map[0] + 3200);
    lstm_5_output_array.data_start = AI_PTR(activations_map[0] + 3200);
    dense_3_output_array.data = AI_PTR(activations_map[0] + 0);
    dense_3_output_array.data_start = AI_PTR(activations_map[0] + 0);
    dense_3_nl_output_array.data = AI_PTR(activations_map[0] + 120);
    dense_3_nl_output_array.data_start = AI_PTR(activations_map[0] + 120);
    dense_4_output_array.data = AI_PTR(activations_map[0] + 0);
    dense_4_output_array.data_start = AI_PTR(activations_map[0] + 0);
    dense_4_nl_output_array.data = AI_PTR(activations_map[0] + 80);
    dense_4_nl_output_array.data_start = AI_PTR(activations_map[0] + 80);
    dense_5_output_array.data = AI_PTR(activations_map[0] + 0);
    dense_5_output_array.data_start = AI_PTR(activations_map[0] + 0);
    dense_5_nl_output_array.data = AI_PTR(NULL);
    dense_5_nl_output_array.data_start = AI_PTR(NULL);
    lstm_4_scratch0_array.data = AI_PTR(activations_map[0] + 0);
    lstm_4_scratch0_array.data_start = AI_PTR(activations_map[0] + 0);
    lstm_5_scratch0_array.data = AI_PTR(activations_map[0] + 0);
    lstm_5_scratch0_array.data_start = AI_PTR(activations_map[0] + 0);
    
    return true;
  }
  return false;
}



AI_DECLARE_STATIC
ai_bool har_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)
  AI_UNUSED(net_ctx)

  ai_ptr weights_map[1] = AI_C_ARRAY_INIT;

  if (ai_platform_get_weights_map(weights_map, 1, params)) {
    /* Updating weights with array addresses */
    
    lstm_4_kernel_array.format |= AI_FMT_FLAG_CONST;
    lstm_4_kernel_array.data = AI_PTR(weights_map[0] + 0);
    lstm_4_kernel_array.data_start = AI_PTR(weights_map[0] + 0);
    lstm_4_recurrent_array.format |= AI_FMT_FLAG_CONST;
    lstm_4_recurrent_array.data = AI_PTR(weights_map[0] + 3600);
    lstm_4_recurrent_array.data_start = AI_PTR(weights_map[0] + 3600);
    lstm_4_peephole_array.format |= AI_FMT_FLAG_CONST;
    lstm_4_peephole_array.data = AI_PTR(weights_map[0] + 13600);
    lstm_4_peephole_array.data_start = AI_PTR(weights_map[0] + 13600);
    lstm_4_bias_array.format |= AI_FMT_FLAG_CONST;
    lstm_4_bias_array.data = AI_PTR(weights_map[0] + 13900);
    lstm_4_bias_array.data_start = AI_PTR(weights_map[0] + 13900);
    lstm_5_kernel_array.format |= AI_FMT_FLAG_CONST;
    lstm_5_kernel_array.data = AI_PTR(weights_map[0] + 14300);
    lstm_5_kernel_array.data_start = AI_PTR(weights_map[0] + 14300);
    lstm_5_recurrent_array.format |= AI_FMT_FLAG_CONST;
    lstm_5_recurrent_array.data = AI_PTR(weights_map[0] + 24300);
    lstm_5_recurrent_array.data_start = AI_PTR(weights_map[0] + 24300);
    lstm_5_peephole_array.format |= AI_FMT_FLAG_CONST;
    lstm_5_peephole_array.data = AI_PTR(weights_map[0] + 34300);
    lstm_5_peephole_array.data_start = AI_PTR(weights_map[0] + 34300);
    lstm_5_bias_array.format |= AI_FMT_FLAG_CONST;
    lstm_5_bias_array.data = AI_PTR(weights_map[0] + 34600);
    lstm_5_bias_array.data_start = AI_PTR(weights_map[0] + 34600);
    dense_3_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_3_weights_array.data = AI_PTR(weights_map[0] + 35000);
    dense_3_weights_array.data_start = AI_PTR(weights_map[0] + 35000);
    dense_3_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_3_bias_array.data = AI_PTR(weights_map[0] + 38000);
    dense_3_bias_array.data_start = AI_PTR(weights_map[0] + 38000);
    dense_4_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_4_weights_array.data = AI_PTR(weights_map[0] + 38120);
    dense_4_weights_array.data_start = AI_PTR(weights_map[0] + 38120);
    dense_4_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_4_bias_array.data = AI_PTR(weights_map[0] + 40520);
    dense_4_bias_array.data_start = AI_PTR(weights_map[0] + 40520);
    dense_5_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_5_weights_array.data = AI_PTR(weights_map[0] + 40600);
    dense_5_weights_array.data_start = AI_PTR(weights_map[0] + 40600);
    dense_5_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_5_bias_array.data = AI_PTR(weights_map[0] + 41080);
    dense_5_bias_array.data_start = AI_PTR(weights_map[0] + 41080);
    return true;
  }
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/
AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_har_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_HAR_MODEL_NAME,
      .model_signature   = AI_HAR_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 584116,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}

AI_API_ENTRY
ai_bool ai_har_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_HAR_MODEL_NAME,
      .model_signature   = AI_HAR_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 584116,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}

AI_API_ENTRY
ai_error ai_har_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_har_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_har_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_har_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if (!net_ctx) return false;

  ai_bool ok = true;
  ok &= har_configure_weights(net_ctx, params);
  ok &= har_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_har_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_har_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_HAR_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

