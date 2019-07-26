#ifndef SIMPLE_CACHE_SIMPLECACHEIMPL_HPP_
#define SIMPLE_CACHE_SIMPLECACHEIMPL_HPP_

#include <sm/logging.hpp>

#include <boost/serialization/shared_ptr.hpp>

#define _CACHE_TEMPLATE template <typename Key>
#define _CACHE_CLASS SimpleCache<Key>

namespace cache {

  _CACHE_TEMPLATE
  _CACHE_CLASS::SimpleCache(std::size_t maxCacheSize)
      : _maxCacheSize(maxCacheSize)
  {

  }

  _CACHE_TEMPLATE
  inline typename _CACHE_CLASS::iterator _CACHE_CLASS::get(const Key& key)
  {
    bool inserted;
    iterator it;
    std::tie(it, inserted) =  _cache.insert( typename CacheStorage::value_type(key, nullptr) ); // Note: insert will return the existing key if present
    if (inserted && this->size() > _maxCacheSize) {
      this->clean(it); // we want to keep the iterator returned valid
      if (this->size() > _maxCacheSize)
        SM_WARN_STREAM("SimpleCache: Cache size " << this->size() << " beyond limit " << _maxCacheSize <<
                       ", mark some cache entries as invalid or increase maximum cache size!");
    }
    return it;
  }
  
  _CACHE_TEMPLATE
  inline typename _CACHE_CLASS::const_iterator _CACHE_CLASS::getConst(const Key& key) const
  {
    const_iterator it = _cache.find(key);
    return it;
  }

  _CACHE_TEMPLATE
  inline void _CACHE_CLASS::clean()
  {
    for (auto it = _cache.begin(); it != _cache.end();) {
      if (it->second == nullptr || !it->second->isValid())
        it = _cache.erase(it);
      else
        ++it;
    }
  }

  _CACHE_TEMPLATE
  inline void _CACHE_CLASS::clean(iterator it_)
  {
    for (auto it = _cache.begin(); it != _cache.end();) {
      if (it != it_ && (it->second == nullptr || !it->second->isValid()))
        it = _cache.erase(it);
      else
        ++it;
    }
  }

  _CACHE_TEMPLATE
  inline std::size_t _CACHE_CLASS::size() const
  {
    return _cache.size();
  }
  
  _CACHE_TEMPLATE
  inline bool _CACHE_CLASS::empty() const
  {
    return _cache.empty();
  }
  
  _CACHE_TEMPLATE
  template<class Archive>
  inline void _CACHE_CLASS::load(Archive & ar, const unsigned int /*version*/) {
    std::size_t numEntries;
    ar >> numEntries;
    Key key;
    typename CacheStorage::mapped_type entry;
    for (std::size_t i=0; i<numEntries; ++i) {
      ar >> key;
      ar >> entry;
      _cache.insert( typename CacheStorage::value_type(key, entry));
    }
  }

  _CACHE_TEMPLATE
  template<class Archive>
  inline void _CACHE_CLASS::save(Archive & ar, const unsigned int /*version*/) const {
    std::size_t numEntries = _cache.size();
    ar << numEntries;
    for (const auto& entry : _cache) {
      ar << entry.first;
      ar << entry.second;
    }
  }

} /* namespace cache */

#undef _MAP_TEMPLATE
#undef _MAP_CLASS

#endif /* SIMPLE_CACHE_SIMPLECACHEIMPL_HPP_ */
