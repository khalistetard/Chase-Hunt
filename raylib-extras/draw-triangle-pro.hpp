#ifndef _DRAW_TRIANGLE_PRO_
#define _DRAW_TRIANGLE_PRO_

#include "rlgl.h"

namespace ai {

// Based on DrawTriangle with translation/rotation a la. DrawPoly
// NOTE: Vertex must be provided in counter-clockwise order
void DrawTrianglePro(Vector2 center, Vector2 v1, Vector2 v2, Vector2 v3,
                     float rotation, Color color)
{
  if (rlCheckBufferLimit(4)) rlglDraw();

  rlPushMatrix();
    rlTranslatef(center.x, center.y, 0.0f);
    rlRotatef(rotation, 0.0f, 0.0f, 1.0f);



#if defined(SUPPORT_QUADS_DRAW_MODE)
    rlEnableTexture(GetShapesTexture().id);

    rlBegin(RL_QUADS);
      rlColor4ub(color.r, color.g, color.b, color.a);

      rlTexCoord2f(GetShapesTextureRec().x/GetShapesTexture().width, GetShapesTextureRec().y/GetShapesTexture().height);
      rlVertex2f(v1.x, v1.y);

      rlTexCoord2f(GetShapesTextureRec().x/GetShapesTexture().width, (GetShapesTextureRec().y + GetShapesTextureRec().height)/GetShapesTexture().height);
      rlVertex2f(v2.x, v2.y);

      rlTexCoord2f((GetShapesTextureRec().x + GetShapesTextureRec().width)/GetShapesTexture().width, (GetShapesTextureRec().y + GetShapesTextureRec().height)/GetShapesTexture().height);
      rlVertex2f(v2.x, v2.y);

      rlTexCoord2f((GetShapesTextureRec().x + GetShapesTextureRec().width)/GetShapesTexture().width, GetShapesTextureRec().y/GetShapesTexture().height);
      rlVertex2f(v3.x, v3.y);
    rlEnd();

    rlDisableTexture();
#else
    rlBegin(RL_TRIANGLES);
      rlColor4ub(color.r, color.g, color.b, color.a);
      rlVertex2f(v1.x, v1.y);
      rlVertex2f(v2.x, v2.y);
      rlVertex2f(v3.x, v3.y);
    rlEnd();
#endif

  rlPopMatrix();
}

// Draw circle outline (Vector version)
void DrawCircleLinesV(Vector2 position, float radius, Color color)
{
  DrawCircleLines(position.x, position.y, radius, color);
}

} // namespace ai

#endif // _DRAW_TRIANGLE_PRO_
