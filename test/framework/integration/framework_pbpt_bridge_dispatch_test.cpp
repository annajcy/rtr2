#include <gtest/gtest.h>

#include "rtr/framework/integration/pbpt/bridge/dispatch.hpp"
#include <string_view>
#include <tuple>
#include <stdexcept>
#include <string>

namespace rtr::framework::integration {
namespace {

struct DummyContext {};
struct DummyRecord {
    int id;
};
struct DummyPackage {};

struct MapperA {
    static constexpr std::string_view kName = "MapperA";
    static bool matches(const DummyRecord& r, const DummyContext&, DummyPackage&) { return r.id == 1; }
    static void map(const DummyRecord&, const DummyContext&, DummyPackage&) {}
};

struct MapperB {
    static constexpr std::string_view kName = "MapperB";
    static bool matches(const DummyRecord& r, const DummyContext&, DummyPackage&) { return r.id == 2; }
    static void map(const DummyRecord&, const DummyContext&, DummyPackage&) {
        throw std::runtime_error("MapperB Error");
    }
};

struct MapperC {
    static constexpr std::string_view kName = "MapperC";
    static bool                       matches(const DummyRecord& r, const DummyContext&, DummyPackage&) {
        if (r.id == 4)
            throw std::runtime_error("MapperC matches Error");
        return false;
    }
    static void map(const DummyRecord&, const DummyContext&, DummyPackage&) {}
};

using TestMapperList = std::tuple<MapperA, MapperC, MapperB>;

TEST(PbptBridgeDispatchTest, FirstMatchWins) {
    DummyContext ctx;
    DummyPackage pkg;

    // Match A
    auto res1 = dispatch_impl(TestMapperList{}, DummyRecord{1}, ctx, pkg);
    EXPECT_TRUE(res1.matched);
    EXPECT_EQ(res1.mapper_name, "MapperA");
}

TEST(PbptBridgeDispatchTest, NoMatchFallback) {
    DummyContext ctx;
    DummyPackage pkg;

    auto res = dispatch_impl(TestMapperList{}, DummyRecord{3}, ctx, pkg);
    EXPECT_FALSE(res.matched);
    EXPECT_EQ(res.mapper_name, "");
}

TEST(PbptBridgeDispatchTest, ExceptionLoggingWrapper) {
    DummyContext ctx;
    DummyPackage pkg;

    try {
        dispatch_impl(TestMapperList{}, DummyRecord{2}, ctx, pkg);
        FAIL() << "Expected exception";
    } catch (const std::exception& e) {
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("[mapper=MapperB]") != std::string::npos);
        EXPECT_TRUE(msg.find("MapperB Error") != std::string::npos);
    }
}

TEST(PbptBridgeDispatchTest, MatchesExceptionLoggingWrapper) {
    DummyContext ctx;
    DummyPackage pkg;

    try {
        dispatch_impl(TestMapperList{}, DummyRecord{4}, ctx, pkg);
        FAIL() << "Expected exception from matches";
    } catch (const std::exception& e) {
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("[mapper=MapperC]") != std::string::npos);
        EXPECT_TRUE(msg.find("matches failed: MapperC matches Error") != std::string::npos);
    }
}

}  // namespace
}  // namespace rtr::framework::integration
