#include "huffman.h"

void HuffmanTree::Build(const std::vector<uint8_t> &code_lengths,
                        const std::vector<uint8_t> &values) {
    if (code_lengths.size() > 16) {
        throw std::invalid_argument("Incorrect tree size!");
    }

    head_ = std::make_shared<Node>();
    state_ = head_;

    std::vector<std::shared_ptr<Node>> unfinished_nodes;
    unfinished_nodes.push_back(head_);

    size_t value_index = 0;

    for (size_t i = 0; i < code_lengths.size(); ++i) {
        std::vector<std::shared_ptr<Node>> new_nodes;

        for (size_t j = 0; j < unfinished_nodes.size(); ++j) {
            std::shared_ptr<Node> left = std::make_shared<Node>();
            std::shared_ptr<Node> right = std::make_shared<Node>();

            unfinished_nodes[j]->left_ = left;
            unfinished_nodes[j]->right_ = right;

            new_nodes.push_back(left);
            new_nodes.push_back(right);
        }

        unfinished_nodes.clear();

        if (code_lengths[i] > new_nodes.size()) {
            throw std::invalid_argument("Wrong amount of nodes!");
        }

        for (size_t j = 0; j < new_nodes.size(); ++j) {
            if (j < code_lengths[i]) {
                if (value_index >= values.size()) {
                    throw std::invalid_argument("Not enough values!");
                }

                new_nodes[j]->value_ = values[value_index++];
                new_nodes[j]->has_value_ = true;
            } else {
                unfinished_nodes.push_back(new_nodes[j]);
            }
        }
    }
}

bool HuffmanTree::Move(bool bit, int &value) {
    if (!head_) {
        throw std::invalid_argument("Tree not built!");
    }

    if (!bit) {
        if (state_->left_) {
            state_ = state_->left_;

            if (state_->has_value_) {
                value = state_->value_;

                state_ = head_;
                return true;
            } else {
                return false;
            }
        } else {
            throw std::invalid_argument("Such node doesn't exist!");
        }
    } else {
        if (state_->right_) {
            state_ = state_->right_;

            if (state_->has_value_) {
                value = state_->value_;

                state_ = head_;
                return true;
            } else {
                return false;
            }
        } else {
            throw std::invalid_argument("Such node doesn't exist!");
        }
    }
}
