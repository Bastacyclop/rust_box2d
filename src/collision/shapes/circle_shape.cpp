b2CircleShape* CircleShape_new() {
    return new b2CircleShape();
}
void CircleShape_drop(b2CircleShape* self) {
    delete self;
}

b2Shape* CircleShape_as_shape(b2CircleShape* self) {
    return static_cast<b2Shape*>(self);
}
b2CircleShape* Shape_as_circle_shape(b2Shape* self) {
    return static_cast<b2CircleShape*>(self);
}

i32 CircleShape_get_support(const b2CircleShape* self,
                            const b2Vec2* d) {
    return self->GetSupport(*d);
}
const b2Vec2* CircleShape_get_support_vertex(const b2CircleShape* self,
                                             const b2Vec2* d) {
    return &self->GetSupportVertex(*d);
}
i32 CircleShape_get_vertex_count(const b2CircleShape* self) {
    return self->GetVertexCount();
}
const b2Vec2* CircleShape_get_vertex(const b2CircleShape* self,
                                     i32 index) {
    return &self->GetVertex(index);
}
