#ifndef __update_history_h__
#define __update_history_h__

#include <map>
#include <stl_utils/stlmap_utils.hpp>
#include <stdint.h>

#include "rbis_update_interface.hpp"

namespace pronto {

class updateHistory {
public:   
   // MXR::note:
   // multimap  关键字可以重复出现的map
  typedef std::multimap<int64_t, RBISUpdateInterface *> historyMap;
  typedef historyMap::iterator historyMapIterator;
  //MXR::note:
  //pair是将2个数据组合成一组数据，当需要这样的需求时就可以使用pair
  typedef std::pair<int64_t, RBISUpdateInterface*> historyPair;
  historyMap updateMap;

  updateHistory(RBISUpdateInterface * init);
  ~updateHistory();

  /**
   * add object to history
   *
   * returns iterator to obj in the history map
   */
  historyMapIterator addToHistory(RBISUpdateInterface * rbisu);

  /**
   * Clear everything in the history older than the given timestamp
   */
  void clearHistoryBeforeUtime(int64_t utime);

  std::string toString() const;
  std::string toString(uint64_t utime, int pos_shift = 1) const;

};

}

#endif
