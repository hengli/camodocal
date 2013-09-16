#include "../dbow2/DBoW2/DBoW2.h"
#include "../dbow2/DUtils/DUtils.h"
#include "../dbow2/DUtilsCV/DUtilsCV.h"
#include "../dbow2/DVision/DVision.h"

int
main(int argc, char** argv)
{
    std::vector<std::vector<std::vector<float> > > features;

    const int k = 10;
    const int L = 5;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L2_NORM;

    Surf64Vocabulary voc(k, L, weight, score);
    voc.create(features);

    voc.save("vocabulary.voc");

    return 0;
}
