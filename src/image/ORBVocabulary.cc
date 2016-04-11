#include "ORBVocabulary.h"

namespace lqlslam {

ORBVocabulary* voc;

bool loadVocabulary(const string &vocFile) {
    voc = new ORBVocabulary();
    bool read = voc->loadFromTextFile(vocFile);
    if (read) {
        return true;
    } else {
        delete voc;
        voc = NULL;
        return false;
    }
}

}
