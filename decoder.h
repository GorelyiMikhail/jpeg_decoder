#pragma once

#include <cstdint>
#include "huffman.h"
#include <istream>
#include <unordered_map>
#include "utils/image.h"
#include <vector>

const size_t k2ndByte = 1 << 8;
const size_t kHalfByte = 1 << 4;

struct MyJpeg {
    size_t precision_;
    size_t height_;
    size_t width_;
    size_t H_max_ = 0;
    size_t V_max_ = 0;
    size_t channels_amount_;
    std::string comment_;
    std::vector<std::vector<double>> Y_, Cb_, Cr_;
    std::vector<std::vector<double>> temp_Y_, temp_Cb_, temp_Cr_;
    std::unordered_map<size_t, size_t> channel_id_to_quantization_id_;
    std::unordered_map<size_t, size_t> channel_id_to_horizontal_thinning_;
    std::unordered_map<size_t, size_t> channel_id_to_vertical_thinning_;
    std::unordered_map<size_t, size_t> channel_id_to_huffman_id_DC_;
    std::unordered_map<size_t, size_t> channel_id_to_huffman_id_AC_;
    std::unordered_map<size_t, HuffmanTree> huffman_id_to_huffman_tree_DC_;
    std::unordered_map<size_t, HuffmanTree> huffman_id_to_huffman_tree_AC_;
    std::unordered_map<size_t, std::vector<int>> quantization_id_to_quantization_matrix_;
};

class Segment {
public:
    virtual void Work(std::istream& input, MyJpeg& jpeg) = 0;

    virtual ~Segment() = default;
};

class StartSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class CommentSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class APPnSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class QuantizationSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class SOF0Segment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class DHTSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class SOSSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

class EndSegment : public Segment {
public:
    void Work(std::istream& input, MyJpeg& jpeg) override;
};

template <class T>
bool Is(const std::shared_ptr<Segment>& obj) {
    if (!std::dynamic_pointer_cast<T>(obj)) {
        return false;
    } else {
        return true;
    }
}

std::unordered_map<size_t, std::pair<size_t, size_t>> ZigZag(size_t n, size_t m);

Image Decode(std::istream& input);

void ReadFile(std::istream& input, MyJpeg& jpeg);

std::shared_ptr<Segment> ReadNextSegment(std::istream& input);

void Quantization(std::vector<std::vector<double>>& result, std::vector<int>& quantization_table);

void IDCT(size_t index, MyJpeg& jpeg, std::vector<std::vector<double>>& from,
          std::vector<std::vector<double>>& to,
          std::unordered_map<size_t, std::pair<size_t, size_t>>& f);

void RgbFunc(MyJpeg& jpeg, Image& image);
