#ifndef SIMPLE_CACHE_CACHEENTRYINTERFACE_HPP_
#define SIMPLE_CACHE_CACHEENTRYINTERFACE_HPP_

namespace cache {
  
  class CacheEntryInterface 
  {
    public:
      CacheEntryInterface() { }
      virtual ~CacheEntryInterface() { }
      
      /// \brief Implement to indicate whether this cache item is still needed
      virtual bool isValid() const = 0;

      template<class Archive>
      inline void serialize(Archive & /*ar*/, const unsigned int /*version*/) { }
  };

} /* namespace cache */

#endif /* SIMPLE_CACHE_CACHEENTRYINTERFACE_HPP_ */
