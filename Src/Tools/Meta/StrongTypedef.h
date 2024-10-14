// Author: Daniele Affinita

#pragma once

#define PHANTOM_TYPE(TYPE) struct TYPE {};

/* 
    Creates a new type, using a unique tag to avoid conflicts.
    It keep the new type separated from the original one
    while allowing conversion if needed.
*/
#define STRONG_TYPEDEF(NEW_TYPE, BASE_TYPE)                        \
    PHANTOM_TYPE(NEW_TYPE##_TAG)                                   \
    using NEW_TYPE = strong_typedef<NEW_TYPE##_TAG, BASE_TYPE>;

/* 
    Defines a type with strict typedef features, 
    ensuring no conversion back to the base type. 
*/
#define STRICT_TYPEDEF(NEW_TYPE, BASE_TYPE)                         \
    PHANTOM_TYPE(NEW_TYPE##_TAG)                                    \
    using NEW_TYPE = strict_typedef<NEW_TYPE##_TAG, BASE_TYPE>;

template <class Tag, typename T>
class strong_typedef : public T {
public:
    using T::T; 

    // Create subtype from original type T, some sort of explicit copy constructor + casting
    strong_typedef(const T& value) : T(value) {}

    // Handling the casting back to the original datatype T.
    operator T&() noexcept { return *this; }
    operator const T&() const noexcept { return *this; }
};

/*
Explicitly disable conversion operators to the base type.
No casting back to base type allowed.
Prevents any mixing of types.
*/
template <class Tag, typename T>
class strict_typedef : public T {
public:
    using T::T; 

    explicit strict_typedef(const T& value) : T(value) {}

    operator T&() = delete;
    operator const T&() = delete;
};
