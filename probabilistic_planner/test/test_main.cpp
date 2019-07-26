#include <string>

#include <gtest/gtest.h>

#include <sm/logging.hpp>
#include <sm/BoostPropertyTree.hpp>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  sm::BoostPropertyTree::setHumanReadableInputOutput(true);
  testing::InitGoogleTest(&argc, argv);
  for (int i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-v") == 0 || strcmp(argv[i],"--verbosity") == 0) && i+1 < argc)
      sm::logging::setLevel(sm::logging::levels::fromString(argv[i+1]));
  }
  return RUN_ALL_TESTS();
}
