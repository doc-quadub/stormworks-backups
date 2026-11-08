#include "value.h"
#include <fstream>
#include <sstream>
#include <cstring>

Value* Dict::find(const std::string& k) {
    auto it = index.find(k);
    return it == index.end() ? nullptr : &items[it->second].second;
}
const Value* Dict::find(const std::string& k) const {
    auto it = index.find(k);
    return it == index.end() ? nullptr : &items[it->second].second;
}
Value& Dict::at(const std::string& k) {
    auto it = index.find(k);
    if (it != index.end()) return items[it->second].second;
    index[k] = items.size();
    items.emplace_back(k, Value());
    return items.back().second;
}
void Dict::set(const std::string& k, Value v) { at(k) = std::move(v); }
bool Dict::erase(const std::string& k) {
    auto it = index.find(k);
    if (it == index.end()) return false;
    size_t pos = it->second;
    items.erase(items.begin() + pos);
    index.erase(it);
    for (auto& kv : index) if (kv.second > pos) kv.second--;
    return true;
}

namespace json {

struct Parser {
    const char* p;
    const char* end;
    void ws() { while (p < end && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) p++; }
    [[noreturn]] void fail(const char* what) { throw std::runtime_error(std::string("json: ") + what); }
    Value value() {
        ws();
        if (p >= end) fail("unexpected end");
        char c = *p;
        if (c == '{') {
            p++;
            Value d = Value::dict();
            ws();
            if (*p == '}') { p++; return d; }
            while (true) {
                ws();
                std::string k = string();
                ws();
                if (*p != ':') fail("expected :");
                p++;
                Value v = value();
                d.dct().set(k, std::move(v));
                ws();
                if (*p == ',') { p++; continue; }
                if (*p == '}') { p++; break; }
                fail("expected , or }");
            }
            return d;
        }
        if (c == '[') {
            p++;
            Value l = Value::list();
            ws();
            if (*p == ']') { p++; return l; }
            while (true) {
                l.append(value());
                ws();
                if (*p == ',') { p++; continue; }
                if (*p == ']') { p++; break; }
                fail("expected , or ]");
            }
            return l;
        }
        if (c == '"') return Value(string());
        if (!strncmp(p, "true", 4)) { p += 4; return Value::Bool(true); }
        if (!strncmp(p, "false", 5)) { p += 5; return Value::Bool(false); }
        if (!strncmp(p, "null", 4)) { p += 4; return Value(); }
        if (!strncmp(p, "NaN", 3)) { p += 3; return Value(NAN); }
        if (!strncmp(p, "Infinity", 8)) { p += 8; return Value(INFINITY); }
        if (!strncmp(p, "-Infinity", 9)) { p += 9; return Value(-INFINITY); }
        const char* start = p;
        bool is_float = false;
        if (*p == '-') p++;
        while (p < end && ((*p >= '0' && *p <= '9') || *p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-')) {
            if (*p == '.' || *p == 'e' || *p == 'E') is_float = true;
            p++;
        }
        std::string num(start, p);
        if (num.empty() || num == "-") fail("bad value");
        if (is_float) return Value(std::strtod(num.c_str(), nullptr));
        return Value((long long)std::strtoll(num.c_str(), nullptr, 10));
    }
    std::string string() {
        if (*p != '"') fail("expected string");
        p++;
        std::string out;
        while (p < end && *p != '"') {
            if (*p == '\\') {
                p++;
                switch (*p) {
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    case '/': out += '/'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'n': out += '\n'; break;
                    case 'r': out += '\r'; break;
                    case 't': out += '\t'; break;
                    case 'u': {
                        unsigned cp = std::strtoul(std::string(p + 1, p + 5).c_str(), nullptr, 16);
                        p += 4;
                        if (cp < 0x80) out += (char)cp;
                        else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
                        else { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
                        break;
                    }
                    default: fail("bad escape");
                }
                p++;
            } else {
                out += *p++;
            }
        }
        p++;
        return out;
    }
};

Value parse(const std::string& text) {
    Parser ps{text.data(), text.data() + text.size()};
    return ps.value();
}

Value load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    return parse(ss.str());
}

static void put_string(std::string& out, const std::string& s) {
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                if (c < 0x20 || c >= 0x7f) {
                    char b[8];
                    std::snprintf(b, sizeof b, "\\u%04x", c);   // (ASCII data only in practice)
                    out += b;
                } else {
                    out += (char)c;
                }
        }
    }
    out += '"';
}

static void put(std::string& out, const Value& v) {
    switch (v.type) {
        case Value::NONE: out += "null"; break;
        case Value::BOOL: out += v.b ? "true" : "false"; break;
        case Value::INT: out += std::to_string(v.i); break;
        case Value::FLOAT:
            if (std::isnan(v.f)) out += "NaN";
            else if (std::isinf(v.f)) out += v.f > 0 ? "Infinity" : "-Infinity";
            else out += py::repr(v.f);
            break;
        case Value::STR: put_string(out, *v.s); break;
        case Value::LIST: {
            out += '[';
            bool first = true;
            for (const Value& x : *v.l) {
                if (!first) out += ", ";
                first = false;
                put(out, x);
            }
            out += ']';
            break;
        }
        case Value::DICT: {
            out += '{';
            bool first = true;
            for (const auto& kv : v.d->items) {
                if (!first) out += ", ";
                first = false;
                put_string(out, kv.first);
                out += ": ";
                put(out, kv.second);
            }
            out += '}';
            break;
        }
    }
}

std::string dumps(const Value& v) {
    std::string out;
    put(out, v);
    return out;
}

void dump(const Value& v, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    std::string s = dumps(v);
    f.write(s.data(), s.size());
}

}  // namespace json
