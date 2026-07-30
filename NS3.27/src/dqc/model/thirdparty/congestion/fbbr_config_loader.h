#ifndef FBBR_CONFIG_LOADER_H_
#define FBBR_CONFIG_LOADER_H_

#include <string>

namespace dqc {

struct FBBRConfig;

// Loads a complete FBBR key=value configuration. On failure, |error| explains
// the first invalid line and |config| is left unchanged.
bool LoadFBBRConfigFile(const std::string& path,
                        FBBRConfig* config,
                        std::string* error = nullptr);

}  // namespace dqc

#endif  // FBBR_CONFIG_LOADER_H_
