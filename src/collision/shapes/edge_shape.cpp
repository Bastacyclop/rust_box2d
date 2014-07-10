b2EdgeShape* EdgeShape_new() {
    return new b2EdgeShape();
}
void EdgeShape_drop(b2EdgeShape* self) {
    delete self;
}

b2Shape* EdgeShape_as_shape(b2EdgeShape* self) {
    return static_cast<b2Shape*>(self);
}
b2EdgeShape* Shape_as_edge_shape(b2Shape* self) {
    return static_cast<b2EdgeShape*>(self);
}

void EdgeShape_set(b2EdgeShape* self,
                   const b2Vec2* v1, const b2Vec2* v2) {
    self->Set(*v1, *v2);
}
