b2PolygonShape PolygonShape_new() {
    return b2PolygonShape();
}

b2Shape* PolygonShape_as_shape(b2PolygonShape* self) {
    return static_cast<b2Shape*>(self);
}
b2PolygonShape* Shape_as_polygon_shape(b2Shape* self) {
    return static_cast<b2PolygonShape*>(self);
}

void PolygonShape_set(b2PolygonShape* self,
                      const b2Vec2* points, i32 count) {
    self->Set(points, count);
}
void PolygonShape_set_as_box(b2PolygonShape* self, f32 hw, f32 hh) {
    self->SetAsBox(hw, hh);
}
void PolygonShape_set_as_oriented_box(b2PolygonShape* self,
                                      f32 hw, f32 hh,
                                      const b2Vec2* center,
                                      f32 angle) {
    self->SetAsBox(hw, hh, *center, angle);
}
i32 PolygonShape_get_vertex_count(const b2PolygonShape* self) {
    return self->GetVertexCount();
}
const b2Vec2* PolygonShape_get_vertex(const b2PolygonShape* self,
                                      i32 index) {
    return &self->GetVertex(index);
}
bool PolygonShape_validate(const b2PolygonShape* self) {
    return self->Validate();
}
