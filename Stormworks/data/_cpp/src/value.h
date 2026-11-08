// A small dynamic value with Python reference semantics (lists and dicts are shared, dicts keep
// insertion order) and json.load / json.dump compatible with CPython's json module defaults.
#pragma once
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>
#include <unordered_map>
#include "py.h"

struct Value;
using List = std::vector<Value>;

struct Dict {
    std::vector<std::pair<std::string, Value>> items;
    std::unordered_map<std::string, size_t> index;     // key -> position in items
    Value* find(const std::string& k);
    const Value* find(const std::string& k) const;
    Value& at(const std::string& k);                    // insert None if missing
    void set(const std::string& k, Value v);
    bool erase(const std::string& k);
    size_t size() const { return items.size(); }
};

struct Value {
    enum Type { NONE, BOOL, INT, FLOAT, STR, LIST, DICT } type = NONE;
    bool b = false;
    long long i = 0;
    double f = 0.0;
    std::shared_ptr<std::string> s;
    std::shared_ptr<List> l;
    std::shared_ptr<Dict> d;

    Value() {}
    Value(std::nullptr_t) {}
    static Value Bool(bool v) { Value x; x.type = BOOL; x.b = v; return x; }
    Value(int v) : type(INT), i(v) {}
    Value(long long v) : type(INT), i(v) {}
    Value(size_t v) : type(INT), i((long long)v) {}
    Value(double v) : type(FLOAT), f(v) {}
    Value(const char* v) : type(STR), s(std::make_shared<std::string>(v)) {}
    Value(const std::string& v) : type(STR), s(std::make_shared<std::string>(v)) {}
    static Value list() { Value x; x.type = LIST; x.l = std::make_shared<List>(); return x; }
    static Value list(std::initializer_list<Value> v) { Value x; x.type = LIST; x.l = std::make_shared<List>(v); return x; }
    static Value dict() { Value x; x.type = DICT; x.d = std::make_shared<Dict>(); return x; }

    bool is_none() const { return type == NONE; }
    bool is_num() const { return type == INT || type == FLOAT || type == BOOL; }
    double num() const {
        if (type == FLOAT) return f;
        if (type == INT) return (double)i;
        if (type == BOOL) return b ? 1.0 : 0.0;
        throw std::runtime_error("not a number");
    }
    long long integer() const {
        if (type == INT) return i;
        if (type == BOOL) return b ? 1 : 0;
        throw std::runtime_error("not an int");
    }
    const std::string& str() const { if (type != STR) throw std::runtime_error("not a str"); return *s; }
    List& lst() const { if (type != LIST) throw std::runtime_error("not a list"); return *l; }
    Dict& dct() const { if (type != DICT) throw std::runtime_error("not a dict"); return *d; }
    // Python truthiness
    bool truthy() const {
        switch (type) {
            case NONE: return false;
            case BOOL: return b;
            case INT: return i != 0;
            case FLOAT: return f != 0.0;
            case STR: return !s->empty();
            case LIST: return !l->empty();
            case DICT: return d->size() > 0;
        }
        return false;
    }
    // dict access
    Value& operator[](const std::string& k) { return dct().at(k); }
    Value& operator[](const char* k) { return dct().at(k); }
    Value get(const std::string& k, Value dflt = Value()) const {
        const Value* v = dct().find(k);
        return v ? *v : dflt;
    }
    bool has(const std::string& k) const { return dct().find(k) != nullptr; }
    // list access
    Value& operator[](size_t idx) { return lst()[idx]; }
    Value& operator[](int idx) { return lst()[idx]; }
    size_t size() const { return type == LIST ? l->size() : type == DICT ? d->size() : 0; }
    void append(Value v) { lst().push_back(std::move(v)); }
    // shallow copy of a dict or list (Python dict(x) / list(x))
    Value copy() const {
        Value x = *this;
        if (type == LIST) x.l = std::make_shared<List>(*l);
        if (type == DICT) x.d = std::make_shared<Dict>(*d);
        return x;
    }
};

namespace json {
Value parse(const std::string& text);
Value load(const std::string& path);
std::string dumps(const Value& v);
void dump(const Value& v, const std::string& path);
}

// helpers
inline Value vec3(double a, double b, double c) { return Value::list({Value(a), Value(b), Value(c)}); }
inline Value vec2(double a, double b) { return Value::list({Value(a), Value(b)}); }
