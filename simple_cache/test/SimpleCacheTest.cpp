#include <gtest/gtest.h>

// standard
#include <string>

// self
#include <simple_cache/SimpleCache.hpp>

// Schweizer Messer
#include <sm/timing/Timer.hpp>

// boost
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace std;
using namespace cache;

struct TestCacheEntry : public CacheEntryInterface
{
  int data = 0;
  virtual ~TestCacheEntry() { }
  bool isValid() const override { return data == 0; }
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    boost::serialization::void_cast_register<TestCacheEntry, CacheEntryInterface>();
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(CacheEntryInterface);
    ar & data;
  }
};

TEST(simple_cache_TESTSUITE, SimpleCache)
{
  try 
  {
    SimpleCache<string> cache(1);
    auto entry = boost::shared_ptr<TestCacheEntry>(new TestCacheEntry());

    {
      auto e = cache.get("test")->second;
      EXPECT_TRUE(e == nullptr);
      EXPECT_EQ(1, cache.size());
      EXPECT_FALSE(cache.empty());
      cache.clean(); // should delete empty entry
      EXPECT_EQ(0, cache.size());
    }
    
    {
      cache.get("test")->second = entry;
      EXPECT_EQ(1, cache.size());
      cache.clean(); // should not delete valid entry
      EXPECT_EQ(1, cache.size());
    }
    
    {
      auto cacheItem = boost::dynamic_pointer_cast<TestCacheEntry>(cache.get("test")->second); //Todo (Hannes): make more comfortable
      EXPECT_TRUE(cacheItem != nullptr);
      EXPECT_EQ(0, cacheItem->data);

      entry->data++;
      EXPECT_EQ(1, cacheItem->data);
      
      cache.clean(); // should delete because cache item marks itself invalid
      EXPECT_EQ(0, cache.size());
      EXPECT_TRUE(cache.empty());
    }
    
    {
      // test automatic cleanup by cache size limits

      // this should automatically clean up the cache, since we do not put anything into the cache
      for (size_t i=0; i<cache.getMaxSize()+1; i++)
        cache.get(to_string(i))->second;
      EXPECT_EQ(1, cache.size());
      EXPECT_FALSE(cache.isFull());

      // this should issue a warning and not delete any entries
      entry->data = 0;
      for (size_t i=0; i<cache.getMaxSize()+1; i++)
        cache.get(to_string(i))->second = entry;
      EXPECT_EQ(cache.getMaxSize()+1, cache.size());
      EXPECT_TRUE(cache.isFull());
    }

  } catch (const exception& e) 
  {
    FAIL() << e.what();
  }
}

TEST(simple_cache_TESTSUITE, Serialization)
{
  try
  {
    auto entry = boost::shared_ptr<TestCacheEntry>(new TestCacheEntry());
    SimpleCache<string> cache(1);
    cache.get("test")->second = entry;

    ostringstream os;
    boost::archive::binary_oarchive oa(os);
    oa.register_type<TestCacheEntry>();
    oa << cache;

    stringstream is;
    is << os.str();

    SimpleCache<string> cacheDeserialized(1);
    boost::archive::binary_iarchive ia(is);
    ia.register_type<TestCacheEntry>();
    ia >> cacheDeserialized;

    EXPECT_EQ(cache.size(), cacheDeserialized.size());
    for (const auto& entry : cache) {
      EXPECT_EQ(entry.first, "test");
      ASSERT_NE(entry.second, nullptr);
      auto cacheItem = boost::dynamic_pointer_cast<TestCacheEntry>(cache.get("test")->second); //Todo (Hannes): make more comfortable
      ASSERT_NE(cacheItem, nullptr);
      EXPECT_EQ(cacheItem->data, 0);
    }

  } catch (const exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(simple_cache_TESTSUITE, Profiling)
{

  try
  {
#ifndef NDEBUG
    const size_t numEntries = 10000;
#else
    const size_t numEntries = 1000000;
#endif
    SimpleCache<string> cache;

    {
      sm::timing::Timer timer("access", false);
      for (size_t i=0; i<numEntries; i++)
        cache.get(to_string(i))->second;
      timer.stop();
    }

    sm::timing::Timing::print(cout);

  } catch (const exception& e)
  {
    FAIL() << e.what();
  }
}
