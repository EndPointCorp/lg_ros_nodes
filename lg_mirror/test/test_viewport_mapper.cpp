#include <gtest/gtest.h>
#include "viewport_mapper.h"
#include "lg_common/WindowGeometry.h"

using lg_common::WindowGeometry;
using lg_common::WindowGeometryPtr;

using namespace ViewportMapperTypes;

typedef WindowGeometry::_width_type WGWT;
typedef WindowGeometry::_height_type WGHT;
typedef WindowGeometry::_x_type WGXT;
typedef WindowGeometry::_y_type WGYT;

void tryGeometryString(const std::string& s, WGWT w, WGHT h, WGXT x, WGYT y) {
  WindowGeometryPtr g = ViewportMapper::GeometryFromString(s);
  EXPECT_EQ(w, g->width);
  EXPECT_EQ(h, g->height);
  EXPECT_EQ(x, g->x);
  EXPECT_EQ(y, g->y);
}

void tryBrokenGeometryString(const std::string& s) {
  ASSERT_THROW(
    ViewportMapper::GeometryFromString(s),
    ViewportMapperStringError
  );
}

TEST(ViewportMapperTests, geometryFromString) {
  tryGeometryString("1920x1080+100+200", 1920, 1080, 100, 200);
  tryGeometryString("16384x16384+0+0", 16384, 16384, 0, 0);
  tryGeometryString("800x600-1-2", 800, 600, -1, -2);
  tryGeometryString(" 800x600-1-2 ", 800, 600, -1, -2);

  tryBrokenGeometryString("asdf");
  tryBrokenGeometryString("1024x768");
  tryBrokenGeometryString("1x1-1");
}

TEST(ViewportMapperTests, transformMatrixIdentity) {
  TransformMatrixPtr m_ptr;

  WindowGeometryPtr root(new WindowGeometry);
  root->width = 3840;
  root->height = 2160;

  WindowGeometryPtr viewport(new WindowGeometry);
  viewport->width = root->width;
  viewport->height = root->height;

  m_ptr = ViewportMapper::TransformGeometry(viewport, root);
  TransformMatrix& m = *m_ptr;
  EXPECT_EQ(1.0, m(0, 0));
  EXPECT_EQ(0.0, m(0, 1));
  EXPECT_EQ(0.0, m(0, 2));
  EXPECT_EQ(0.0, m(1, 0));
  EXPECT_EQ(1.0, m(1, 1));
  EXPECT_EQ(0.0, m(1, 2));
  EXPECT_EQ(0.0, m(2, 0));
  EXPECT_EQ(0.0, m(2, 1));
  EXPECT_EQ(1.0, m(2, 2));
}

TEST(ViewportMapperTests, transformMatrixQuad) {
  TransformMatrixPtr m_ptr;

  WindowGeometryPtr root(new WindowGeometry);
  root->width = 3840;
  root->height = 2160;

  WindowGeometryPtr viewport(new WindowGeometry);
  viewport->width = 1920;
  viewport->height = 1080;
  viewport->x = 1920;
  viewport->y = 1080;

  m_ptr = ViewportMapper::TransformGeometry(viewport, root);
  TransformMatrix& m = *m_ptr;
  EXPECT_EQ(0.5, m(0, 0));
  EXPECT_EQ(0.0, m(0, 1));
  EXPECT_EQ(0.5, m(0, 2));
  EXPECT_EQ(0.0, m(1, 0));
  EXPECT_EQ(0.5, m(1, 1));
  EXPECT_EQ(0.5, m(1, 2));
  EXPECT_EQ(0.0, m(2, 0));
  EXPECT_EQ(0.0, m(2, 1));
  EXPECT_EQ(1.0, m(2, 2));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
