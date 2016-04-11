#ifndef ORB_VOCABULARY_H
#define ORB_VOCABULARY_H

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace lqlslam {

/*
 * ORBVocabulary
 */
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>ORBVocabulary;

/*
 * orb vocabulary
 */
extern ORBVocabulary* voc;

/*
 * loadVocabulary
 * @param const string& vocFile, file path of orb vocabulary
 * function: load orb vocabulary and store it in voc
 *         if successfully loaded, voc stores vocabularies and return true;
 *         otherwise, return false
 */
bool loadVocabulary(const string& vocFile);

}

#endif
