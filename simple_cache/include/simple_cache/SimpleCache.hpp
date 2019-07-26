#ifndef SIMPLE_CACHE_SIMPLECACHE_HPP_
#define SIMPLE_CACHE_SIMPLECACHE_HPP_

// standard
#include <unordered_map>
#include <limits>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>

// self
#include <simple_cache/CacheEntryInterface.hpp>

namespace cache {

  template <typename Key>
  class SimpleCache 
  {
    private:
      typedef std::unordered_map<Key, boost::shared_ptr<CacheEntryInterface> > CacheStorage;

    public:
      typedef typename CacheStorage::iterator iterator;
      typedef typename CacheStorage::const_iterator const_iterator;
      
    public:

      SimpleCache() { }
      SimpleCache(std::size_t maxCacheSize);
      ~SimpleCache() { }
      
      /// \brief Returns an iterator to the cache entry with key \p key. The iterator will always be valid.
      inline iterator get(const Key& key);
      /// \brief Returns a const iterator to the cache entry with key \p key. If the key is not present, the iterator will be invalid.
      inline const_iterator getConst(const Key& key) const;
      /// \brief Checks whether an iterator is valid
      inline bool isValid(const const_iterator& it) const { return it != _cache.end(); }
      inline void clean();
      inline std::size_t size() const;
      inline bool empty() const;
      bool isFull() const { return size() > _maxCacheSize; }
      
      std::size_t getMaxSize() const { return _maxCacheSize; }
      void setMaxSize(const std::size_t sz) const { _maxCacheSize = sz; }

      inline iterator begin() { return _cache.begin(); }
      inline const_iterator begin() const { return _cache.begin(); }
      inline iterator end() { return _cache.end(); }
      inline const_iterator end() const { return _cache.end(); }

      // Serialization methods
      BOOST_SERIALIZATION_SPLIT_MEMBER();
      template<class Archive>
      inline void load(Archive & ar, const unsigned int version);
      template<class Archive>
      inline void save(Archive & ar, const unsigned int version) const;

    private:
      /// \brief Clean, but keep the iterator \p it alive
      inline void clean(iterator it_);

    private:
      CacheStorage _cache;
      std::size_t _maxCacheSize = std::numeric_limits<std::size_t>::max();
  };
  
} /* namespace cache */

#include "impl/SimpleCacheImpl.hpp"

#endif /* SIMPLE_CACHE_SIMPLECACHE_HPP_ */
