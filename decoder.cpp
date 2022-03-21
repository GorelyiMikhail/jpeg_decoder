#include "decoder.h"
#include "fft.h"
#include <glog/logging.h>

#include <bitset>
#include <cassert>
#include <cmath>
#include <iostream>

Image Decode(std::istream& input) {
    DLOG(INFO) << "Started decoding!";

    MyJpeg jpeg;

    ReadFile(input, jpeg);

    Image result(jpeg.width_, jpeg.height_);

    DLOG(INFO) << "Started quantization!";

    Quantization(
        jpeg.temp_Y_,
        jpeg.quantization_id_to_quantization_matrix_[jpeg.channel_id_to_quantization_id_[1]]);

    if (jpeg.channels_amount_ > 1) {
        Quantization(
            jpeg.temp_Cb_,
            jpeg.quantization_id_to_quantization_matrix_[jpeg.channel_id_to_quantization_id_[2]]);
    }

    if (jpeg.channels_amount_ > 2) {
        Quantization(
            jpeg.temp_Cr_,
            jpeg.quantization_id_to_quantization_matrix_[jpeg.channel_id_to_quantization_id_[3]]);
    }

    DLOG(INFO) << "Finished quantization!";

    std::unordered_map<size_t, std::pair<size_t, size_t>> f = ZigZag(8, 8);

    DLOG(INFO) << "Started IDCT!";

    IDCT(1, jpeg, jpeg.temp_Y_, jpeg.Y_, f);

    if (jpeg.channels_amount_ > 1) {
        IDCT(2, jpeg, jpeg.temp_Cb_, jpeg.Cb_, f);
    }

    if (jpeg.channels_amount_ > 2) {
        IDCT(3, jpeg, jpeg.temp_Cr_, jpeg.Cr_, f);
    }

    DLOG(INFO) << "Finished IDCT!";

    DLOG(INFO) << "Started RGB!";

    RgbFunc(jpeg, result);

    DLOG(INFO) << "Finished RGB!";

    // for (size_t i = 0; i < jpeg.height_; ++i) {
    //     for (size_t j = 0; j < jpeg.width_; ++j) {
    //         std::cout << result.GetPixel(i, j).r << ' ';
    //     }
    //     std::cout << '\n';
    // }
    // std::cout << '\n';

    result.SetComment(jpeg.comment_);

    DLOG(INFO) << "Finished decoding!";

    return result;
}

void ReadFile(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Started reading file!";

    std::shared_ptr<Segment> current_segment;

    size_t max_soi_count = 1;
    size_t max_sof_count = 1;

    while (!Is<EndSegment>(current_segment)) {
        current_segment = ReadNextSegment(input);

        if (Is<StartSegment>(current_segment)) {
            if (max_soi_count == 0) {
                throw std::invalid_argument("Too many SOI markers!");
            } else {
                --max_soi_count;
            }
        }

        if (Is<SOF0Segment>(current_segment)) {
            if (max_sof_count == 0) {
                throw std::invalid_argument("Too many SOF markers!");
            } else {
                --max_sof_count;
            }
        }

        if (Is<SOSSegment>(current_segment) && (max_soi_count != 0 || max_sof_count != 0)) {
            throw std::invalid_argument("SOS too early!");
        }

        current_segment->Work(input, jpeg);
    }

    int something;

    if (input >> something) {
        throw std::invalid_argument("Some excess data at the end of file!");
    }

    DLOG(INFO) << "Finished reading file!";
}

std::shared_ptr<Segment> ReadNextSegment(std::istream& input) {
    if (input.get() != 0xff) {
        throw std::invalid_argument("That's not a section!");
    }

    size_t segment_id = input.get();

    if (segment_id == 0xd8) {
        return std::make_shared<StartSegment>();
    } else if (segment_id == 0xfe) {
        return std::make_shared<CommentSegment>();
    } else if (0xe0 <= segment_id && segment_id <= 0xef) {
        return std::make_shared<APPnSegment>();
    } else if (segment_id == 0xdb) {
        return std::make_shared<QuantizationSegment>();
    } else if (segment_id == 0xc0) {
        return std::make_shared<SOF0Segment>();
    } else if (segment_id == 0xc4) {
        return std::make_shared<DHTSegment>();
    } else if (segment_id == 0xda) {
        return std::make_shared<SOSSegment>();
    } else if (segment_id == 0xd9) {
        return std::make_shared<EndSegment>();
    } else {
        throw std::invalid_argument("Unknown section!");
    }
}

void StartSegment::Work(std::istream&, MyJpeg&) {
    DLOG(INFO) << "Found StartSegment!";

    DLOG(INFO) << "Finished working on StartSegment!";
}

void CommentSegment::Work(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Found CommentSegment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    if (size <= 2) {
        throw std::invalid_argument("Wrong size!");
    }

    for (size_t i = 0; i < size - 2; ++i) {
        jpeg.comment_.push_back(input.get());
    }

    DLOG(INFO) << "Finished working on CommentSegment!";
}

void APPnSegment::Work(std::istream& input, MyJpeg&) {
    DLOG(INFO) << "Found APPnSegment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    DLOG(INFO) << "APPnSegment of size " << size;

    for (size_t i = 0; i < size - 2; ++i) {
        input.get();
    }

    DLOG(INFO) << "Finished working on APPnSegment!";
}

void QuantizationSegment::Work(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Found QuantizationSegment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    size -= 2;

    while (size > 0) {
        size_t info = input.get();

        size -= 1;

        if ((info / kHalfByte == 0 && size < 64) || (info / kHalfByte == 1 && size < 128)) {
            throw std::invalid_argument("Wrong size!");
        }

        for (size_t i = 0; i < 64; ++i) {
            if (info / kHalfByte == 0) {
                jpeg.quantization_id_to_quantization_matrix_[info % kHalfByte].push_back(
                    input.get());

                size -= 1;
            } else {
                size_t value = input.get() * k2ndByte;
                jpeg.quantization_id_to_quantization_matrix_[info % kHalfByte].push_back(
                    value + input.get());

                size -= 2;
            }
        }
    }

    DLOG(INFO) << "Finished working on QuantizationSegment!";
}

void SOF0Segment::Work(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Found SOF0Segment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    if (size != 11 && size != 14 && size != 17) {
        throw std::invalid_argument("Wrong size!");
    }

    jpeg.precision_ = input.get();

    jpeg.height_ = input.get() * k2ndByte;
    jpeg.height_ += input.get();

    jpeg.width_ = input.get() * k2ndByte;
    jpeg.width_ += input.get();

    jpeg.channels_amount_ = input.get();

    for (size_t i = 0; i < jpeg.channels_amount_; ++i) {
        size_t id = input.get();

        size_t thinning = input.get();
        jpeg.channel_id_to_horizontal_thinning_[id] = thinning / kHalfByte;
        jpeg.channel_id_to_vertical_thinning_[id] = thinning % kHalfByte;

        jpeg.H_max_ = std::max(jpeg.H_max_, thinning / kHalfByte);
        jpeg.V_max_ = std::max(jpeg.V_max_, thinning % kHalfByte);

        jpeg.channel_id_to_quantization_id_[id] = input.get();
    }

    DLOG(INFO) << "Finished working on SOF0Segment!";
}

void DHTSegment::Work(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Found DHTSegment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    size -= 2;

    while (size > 0) {
        if (size < 17) {
            throw std::invalid_argument("Wrong size!");
        }

        size_t info = input.get();
        --size;

        std::vector<uint8_t> code_lengths;

        size_t values_amount = 0;

        for (size_t i = 0; i < 16; ++i) {
            size_t cur_values = input.get();

            values_amount += cur_values;

            code_lengths.push_back(cur_values);

            --size;
        }

        std::vector<uint8_t> values;

        for (size_t i = 0; i < values_amount; ++i) {
            values.push_back(input.get());

            --size;
        }

        HuffmanTree tree;

        tree.Build(code_lengths, values);

        if (info / kHalfByte == 0) {
            jpeg.huffman_id_to_huffman_tree_DC_[info % kHalfByte] = tree;
        } else {
            jpeg.huffman_id_to_huffman_tree_AC_[info % kHalfByte] = tree;
        }

        if (size < 0) {
            throw std::invalid_argument("Wrong size!");
        }
    }

    DLOG(INFO) << "Finished working on DHTSegment!";
}

void SOSSegment::Work(std::istream& input, MyJpeg& jpeg) {
    DLOG(INFO) << "Found SOSSegment!";

    size_t size = input.get() * k2ndByte;
    size += input.get();

    if (size != 8 && size != 10 && size != 12) {
        throw std::invalid_argument("Wrong size!");
    }

    size_t channels_amount = input.get();

    if (channels_amount != jpeg.channels_amount_) {
        throw std::invalid_argument("Wrong channels amount!");
    }

    for (size_t i = 0; i < channels_amount; ++i) {
        size_t id = input.get();

        size_t huffman_id = input.get();
        jpeg.channel_id_to_huffman_id_DC_[id] = huffman_id / kHalfByte;
        jpeg.channel_id_to_huffman_id_AC_[id] = huffman_id % kHalfByte;
    }

    for (size_t i = 0; i < 3; ++i) {
        size_t num = input.get();
        if ((i == 0 && num != 0) || (i == 1 && num != 63) || (i == 2 && num != 0)) {
            throw std::invalid_argument("Wrong progressive thing!");
        }
    }

    enum State { TREE, READ };

    enum Matrix { Y, Cb, Cr };

    State cur_state = TREE;

    Matrix cur_matrix = Y;

    size_t zeroes = 0;
    size_t to_read = 0;
    size_t cur_size = 0;
    double element = 0;
    bool inverted = false;
    bool continue_next = false;

    size_t mc_us_x_amount = std::ceil(
        static_cast<double>(std::ceil(static_cast<double>(jpeg.width_) / jpeg.H_max_)) / 8);
    size_t mc_us_y_amount = std::ceil(
        static_cast<double>(std::ceil(static_cast<double>(jpeg.height_) / jpeg.V_max_)) / 8);

    size_t y_index = 0;
    size_t y_x_amount = jpeg.H_max_ / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[1]);
    size_t y_y_amount = jpeg.V_max_ / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[1]);
    jpeg.temp_Y_.resize(y_x_amount * y_y_amount * mc_us_x_amount * mc_us_y_amount);

    size_t cb_index = 0;
    size_t cb_x_amount = 0;
    size_t cb_y_amount = 0;
    if (jpeg.channels_amount_ >= 2) {
        cb_x_amount = jpeg.H_max_ / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[2]);
        cb_y_amount = jpeg.V_max_ / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[2]);
        jpeg.temp_Cb_.resize(cb_x_amount * cb_y_amount * mc_us_x_amount * mc_us_y_amount);
    }

    size_t cr_index = 0;
    size_t cr_x_amount = 0;
    size_t cr_y_amount = 0;
    if (jpeg.channels_amount_ == 3) {
        cr_index = 0;
        cr_x_amount = jpeg.H_max_ / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[3]);
        cr_y_amount = jpeg.V_max_ / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[3]);
        jpeg.temp_Cr_.resize(cr_x_amount * cr_y_amount * mc_us_x_amount * mc_us_y_amount);
    }

    double last_dc_y = 0;
    double last_dc_cb = 0;
    double last_dc_cr = 0;

    while (1) {
        if (y_index == y_x_amount * y_y_amount * mc_us_x_amount * mc_us_y_amount &&
            cb_index == cb_x_amount * cb_y_amount * mc_us_x_amount * mc_us_y_amount &&
            cr_index == cr_x_amount * cr_y_amount * mc_us_x_amount * mc_us_y_amount) {
            break;
        }

        std::bitset<8> bits = input.get();

        if (continue_next) {
            continue_next = false;
            continue;
        }

        if (bits == 0xff && input.peek() == 0x00) {
            continue_next = true;
        }

        for (size_t i = 0; i < 8; ++i) {
            if (cur_state == TREE) {
                int value;

                if (cur_matrix == Y) {
                    if (y_index < jpeg.temp_Y_.size() && jpeg.temp_Y_[y_index].empty()) {
                        if (jpeg.huffman_id_to_huffman_tree_DC_
                                [jpeg.channel_id_to_huffman_id_DC_[1]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    } else {
                        if (jpeg.huffman_id_to_huffman_tree_AC_
                                [jpeg.channel_id_to_huffman_id_AC_[1]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    }
                } else if (cur_matrix == Cb) {
                    if (cb_index < jpeg.temp_Cb_.size() && jpeg.temp_Cb_[cb_index].empty()) {
                        if (jpeg.huffman_id_to_huffman_tree_DC_
                                [jpeg.channel_id_to_huffman_id_DC_[2]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    } else {
                        if (jpeg.huffman_id_to_huffman_tree_AC_
                                [jpeg.channel_id_to_huffman_id_AC_[2]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    }
                } else {
                    if (cr_index < jpeg.temp_Cr_.size() && jpeg.temp_Cr_[cr_index].empty()) {
                        if (jpeg.huffman_id_to_huffman_tree_DC_
                                [jpeg.channel_id_to_huffman_id_DC_[3]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    } else {
                        if (jpeg.huffman_id_to_huffman_tree_AC_
                                [jpeg.channel_id_to_huffman_id_AC_[3]]
                                    .Move(bits[7 - i], value)) {
                            zeroes = value / kHalfByte;
                            to_read = value % kHalfByte;

                            cur_size = to_read;

                            cur_state = READ;
                        }
                    }
                }

                if (cur_state == READ && cur_matrix == Y && y_index < jpeg.temp_Y_.size() &&
                    jpeg.temp_Y_[y_index].empty() && to_read == 0) {
                    jpeg.temp_Y_[y_index].push_back(last_dc_y);

                    cur_state = TREE;
                }

                if (cur_state == READ && cur_matrix == Cb && cb_index < jpeg.temp_Cb_.size() &&
                    jpeg.temp_Cb_[cb_index].empty() && to_read == 0) {
                    jpeg.temp_Cb_[cb_index].push_back(last_dc_cb);

                    cur_state = TREE;
                }

                if (cur_state == READ && cur_matrix == Cr && cr_index < jpeg.temp_Cr_.size() &&
                    jpeg.temp_Cr_[cr_index].empty() && to_read == 0) {
                    jpeg.temp_Cr_[cr_index].push_back(last_dc_cr);

                    cur_state = TREE;
                }

                if (cur_state == READ && to_read == 0 && zeroes != 0) {
                    for (size_t j = 0; j <= zeroes; ++j) {
                        if (cur_matrix == Y) {
                            jpeg.temp_Y_[y_index].push_back(0);
                        } else if (cur_matrix == Cb) {
                            jpeg.temp_Cb_[cb_index].push_back(0);
                        } else if (cur_matrix == Cr) {
                            jpeg.temp_Cr_[cr_index].push_back(0);
                        }
                    }

                    cur_state = TREE;
                }

                if (cur_state == READ &&
                    ((cur_matrix == Y && y_index < jpeg.temp_Y_.size() &&
                      !jpeg.temp_Y_[y_index].empty()) ||
                     (cur_matrix == Cb && cb_index < jpeg.temp_Cb_.size() &&
                      !jpeg.temp_Cb_[cb_index].empty()) ||
                     (cur_matrix == Cr && cr_index < jpeg.temp_Cr_.size() &&
                      !jpeg.temp_Cr_[cr_index].empty())) &&
                    to_read == 0 && zeroes == 0) {
                    if (cur_matrix == Y) {
                        while (jpeg.temp_Y_[y_index].size() < 64) {
                            jpeg.temp_Y_[y_index].push_back(0);
                        }
                    } else if (cur_matrix == Cb) {
                        while (jpeg.temp_Cb_[cb_index].size() < 64) {
                            jpeg.temp_Cb_[cb_index].push_back(0);
                        }
                    } else if (cur_matrix == Cr) {
                        while (jpeg.temp_Cr_[cr_index].size() < 64) {
                            jpeg.temp_Cr_[cr_index].push_back(0);
                        }
                    }

                    if (cur_matrix == Y) {
                        ++y_index;
                    } else if (cur_matrix == Cb) {
                        ++cb_index;
                    } else {
                        ++cr_index;
                    }

                    if (cur_matrix == Y && y_index % (y_x_amount * y_y_amount) == 0) {
                        if (jpeg.channels_amount_ >= 2) {
                            cur_matrix = Cb;
                        }
                    } else if (cur_matrix == Cb && cb_index % (cb_x_amount * cb_y_amount) == 0) {
                        if (jpeg.channels_amount_ == 3) {
                            cur_matrix = Cr;
                        } else {
                            cur_matrix = Y;
                        }
                    } else if (cur_matrix == Cr && cr_index % (cr_x_amount * cr_y_amount) == 0) {
                        cur_matrix = Y;
                    }

                    cur_state = TREE;
                }
            } else {
                if (zeroes > 0) {
                    for (size_t j = 0; j < zeroes; ++j) {
                        if (cur_matrix == Y) {
                            jpeg.temp_Y_[y_index].push_back(0);
                        } else if (cur_matrix == Cb) {
                            jpeg.temp_Cb_[cb_index].push_back(0);
                        } else if (cur_matrix == Cr) {
                            jpeg.temp_Cr_[cr_index].push_back(0);
                        }
                    }

                    zeroes = 0;
                }

                if (cur_size == to_read && bits[7 - i] == 0) {
                    inverted = true;
                }

                element += bits[7 - i] * (1 << --to_read);

                if (to_read == 0) {
                    if (inverted) {
                        element -= (1 << cur_size) - 1;

                        cur_size = 0;

                        inverted = false;
                    }

                    if (cur_matrix == Y && y_index < jpeg.temp_Y_.size() &&
                        jpeg.temp_Y_[y_index].empty()) {
                        element += last_dc_y;
                        last_dc_y = element;
                    } else if (cur_matrix == Cb && cb_index < jpeg.temp_Cb_.size() &&
                               jpeg.temp_Cb_[cb_index].empty()) {
                        element += last_dc_cb;
                        last_dc_cb = element;
                    } else if (cur_matrix == Cr && cr_index < jpeg.temp_Cr_.size() &&
                               jpeg.temp_Cr_[cr_index].empty()) {
                        element += last_dc_cr;
                        last_dc_cr = element;
                    }

                    if (cur_matrix == Y) {
                        jpeg.temp_Y_[y_index].push_back(element);
                    } else if (cur_matrix == Cb) {
                        jpeg.temp_Cb_[cb_index].push_back(element);
                    } else if (cur_matrix == Cr) {
                        jpeg.temp_Cr_[cr_index].push_back(element);
                    }

                    element = 0;

                    cur_state = TREE;
                }

                if ((cur_matrix == Y && jpeg.temp_Y_[y_index].size() == 64) ||
                    (cur_matrix == Cb && jpeg.temp_Cb_[cb_index].size() == 64) ||
                    (cur_matrix == Cr && jpeg.temp_Cr_[cr_index].size() == 64)) {
                    if (cur_matrix == Y) {
                        ++y_index;
                    } else if (cur_matrix == Cb) {
                        ++cb_index;
                    } else {
                        ++cr_index;
                    }

                    if (cur_matrix == Y && y_index % (y_x_amount * y_y_amount) == 0) {
                        if (jpeg.channels_amount_ >= 2) {
                            cur_matrix = Cb;
                        }
                    } else if (cur_matrix == Cb && cb_index % (cb_x_amount * cb_y_amount) == 0) {
                        if (jpeg.channels_amount_ == 3) {
                            cur_matrix = Cr;
                        } else {
                            cur_matrix = Y;
                        }
                    } else if (cur_matrix == Cr && cr_index % (cr_x_amount * cr_y_amount) == 0) {
                        cur_matrix = Y;
                    }

                    cur_state = TREE;
                }
            }
        }
    }

    DLOG(INFO) << "Finished working on SOSSegment!";
}

void EndSegment::Work(std::istream&, MyJpeg&) {
    DLOG(INFO) << "Found EndSegment!";

    DLOG(INFO) << "Finished working on EndSegment!";
}

// Тут код взят с https://www.geeksforgeeks.org/print-matrix-in-zig-zag-fashion/
std::unordered_map<size_t, std::pair<size_t, size_t>> ZigZag(size_t n, size_t m) {
    std::unordered_map<size_t, std::pair<size_t, size_t>> answer;

    size_t row = 0, col = 0;
    size_t count = 0;

    bool row_inc = 0;

    size_t min = std::min(n, m);
    for (size_t len = 1; len <= min; ++len) {
        for (size_t i = 0; i < len; ++i) {
            answer[count++] = std::make_pair(row, col);

            if (i + 1 == len) {
                break;
            }

            if (row_inc) {
                ++row;
                --col;
            } else {
                --row;
                ++col;
            }
        }

        if (len == min) {
            break;
        }

        if (row_inc) {
            ++row;
            row_inc = false;
        } else {
            ++col;
            row_inc = true;
        }
    }

    if (row == 0) {
        if (col == m - 1) {
            ++row;
        } else {
            ++col;
        }

        row_inc = true;
    } else {
        if (row == n - 1) {
            ++col;
        } else {
            ++row;
        }

        row_inc = false;
    }

    size_t max = std::max(n, m) - 1;
    for (size_t len, diag = max; diag > 0; --diag) {
        if (diag > min) {
            len = min;
        } else {
            len = diag;
        }

        for (size_t i = 0; i < len; ++i) {
            answer[count++] = std::make_pair(row, col);

            if (i + 1 == len) {
                break;
            }

            if (row_inc) {
                ++row, --col;
            } else {
                ++col, --row;
            }
        }

        if (row == 0 || col == m - 1) {
            if (col == m - 1) {
                ++row;
            } else {
                ++col;
            }

            row_inc = true;
        } else if (col == 0 || row == n - 1) {
            if (row == n - 1) {
                ++col;
            } else {
                ++row;
            }

            row_inc = false;
        }
    }

    return answer;
}

void Quantization(std::vector<std::vector<double>>& vec, std::vector<int>& quantization_table) {
    for (size_t i = 0; i < vec.size(); ++i) {
        for (size_t j = 0; j < vec[i].size(); ++j) {
            vec[i][j] *= quantization_table[j];
        }
    }
}

void IDCT(size_t index, MyJpeg& jpeg, std::vector<std::vector<double>>& from,
          std::vector<std::vector<double>>& to,
          std::unordered_map<size_t, std::pair<size_t, size_t>>& f) {
    size_t x_amount = jpeg.channel_id_to_horizontal_thinning_[index];
    size_t y_amount = jpeg.channel_id_to_vertical_thinning_[index];
    auto blocks = ZigZag(y_amount, x_amount);

    size_t x_total = std::ceil(static_cast<double>(jpeg.width_) / jpeg.H_max_ / 8);
    size_t y_total = std::ceil(static_cast<double>(jpeg.height_) / jpeg.V_max_ / 8);

    to.resize(y_amount * y_total * 8, std::vector<double>(x_amount * x_total * 8));

    for (size_t i = 0; i < from.size(); ++i) {
        std::vector<double> rat(64);

        for (size_t j = 0; j < from[i].size(); ++j) {
            rat[f[j].first * 8 + f[j].second] = from[i][j];
        }

        DctCalculator calc(8, &rat, &rat);

        calc.Inverse();

        size_t x = (blocks[i % (x_amount * y_amount)].second +
                    i / (x_amount * y_amount) % x_total * x_amount) *
                   8;
        size_t y = (blocks[i % (x_amount * y_amount)].first +
                    i / (x_amount * y_amount) / x_total * y_amount) *
                   8;

        for (size_t j = 0; j < 8; ++j) {
            for (size_t k = 0; k < 8; ++k) {
                to[y + j][x + k] = std::min(std::max(0.0, std::round(rat[j * 8 + k] + 128)), 255.0);
                // std::cout << to[y + j][x + k] << ' ';
            }
            // std::cout << '\n';
        }
        // std::cout << '\n';
    }
}

void RgbFunc(MyJpeg& jpeg, Image& image) {
    for (size_t i = 0; i < jpeg.height_; ++i) {
        for (size_t j = 0; j < jpeg.width_; ++j) {
            RGB rgb;

            size_t y_x_index = j / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[1]);
            size_t y_y_index = i / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[1]);

            size_t cb_x_index = 0;
            size_t cb_y_index = 0;

            size_t cr_x_index = 0;
            size_t cr_y_index = 0;

            if (jpeg.channels_amount_ <= 2) {
                rgb.r = std::round(jpeg.Y_[y_y_index][y_x_index]);
                rgb.g = std::round(jpeg.Y_[y_y_index][y_x_index]);
                rgb.b = std::round(jpeg.Y_[y_y_index][y_x_index]);
            } else if (jpeg.channels_amount_ == 3) {
                cb_x_index = j / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[2]);
                cb_y_index = i / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[2]);

                cr_x_index = j / (jpeg.H_max_ / jpeg.channel_id_to_horizontal_thinning_[3]);
                cr_y_index = i / (jpeg.V_max_ / jpeg.channel_id_to_vertical_thinning_[3]);

                rgb.r = std::round(jpeg.Y_[y_y_index][y_x_index] +
                                   1.402 * (jpeg.Cr_[cr_y_index][cr_x_index] - 128));
                rgb.g = std::round(jpeg.Y_[y_y_index][y_x_index] -
                                   0.34414 * (jpeg.Cb_[cb_y_index][cb_x_index] - 128) -
                                   0.71414 * (jpeg.Cr_[cr_y_index][cr_x_index] - 128));
                rgb.b = std::round(jpeg.Y_[y_y_index][y_x_index] +
                                   1.772 * (jpeg.Cb_[cb_y_index][cb_x_index] - 128));
            }

            rgb.r = std::min(std::max(0, rgb.r), 255);
            rgb.g = std::min(std::max(0, rgb.g), 255);
            rgb.b = std::min(std::max(0, rgb.b), 255);

            image.SetPixel(i, j, rgb);
        }
        // std::cout << '\n';
    }
}