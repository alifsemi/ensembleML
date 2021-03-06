/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <time.h>

/* Container for time struct */
typedef struct _time_counter {
    /* Current POSIX time in secs. */
    time_t current_secs;
    /* Nanoseconds expired in current second. */
    time_t current_nsecs;
} time_counter;

#endif /* TIMER_H */