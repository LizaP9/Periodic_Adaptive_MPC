#ifndef YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66
#define YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66

#if defined(_MSC_VER) ||                                            \
    (defined(__GNUC__) && (__GNUC__ == 3 && __GNUC_MINOR__ >= 4) || \
     (__GNUC__ >= 4))  // GCC supports "pragma once" correctly since 3.4
#pragma once
#endif

#include "ParamHandler/dynacore_yaml-cpp/parser.h"
#include "ParamHandler/dynacore_yaml-cpp/emitter.h"
#include "ParamHandler/dynacore_yaml-cpp/emitterstyle.h"
#include "ParamHandler/dynacore_yaml-cpp/stlemitter.h"
#include "ParamHandler/dynacore_yaml-cpp/exceptions.h"

#include "ParamHandler/dynacore_yaml-cpp/node/node.h"
#include "ParamHandler/dynacore_yaml-cpp/node/impl.h"
#include "ParamHandler/dynacore_yaml-cpp/node/convert.h"
#include "ParamHandler/dynacore_yaml-cpp/node/iterator.h"
#include "ParamHandler/dynacore_yaml-cpp/node/detail/impl.h"
#include "ParamHandler/dynacore_yaml-cpp/node/parse.h"
#include "ParamHandler/dynacore_yaml-cpp/node/emit.h"

#endif  // YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66
