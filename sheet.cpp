#include "sheet.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>

#include "cell.h"
#include "common.h"
using namespace std::literals;

void Sheet::SetCell(Position pos, std::string text) {
	if (!pos.IsValid()) throw InvalidPositionException("Invalid cell position");

	auto tmp_cell = std::make_unique<Cell>(*this);
	tmp_cell->Set(std::move(text));
	auto cell_refs = tmp_cell->GetReferencedCells();

	if (graph_.DetectCircularDependency(pos, cell_refs)) throw CircularDependencyException("Has circular dependency");

	InvalidateCache(pos);
	graph_.EraseVertex(pos);

	for (auto it = std::make_move_iterator(cell_refs.begin()); it != std::make_move_iterator(cell_refs.end()); ++it) {
		if (!GetCell(*it)) SetCell(*it, "");
		graph_.AddEdge({ pos, *it });
	}

	sheet_[pos.row][pos.col] = std::move(tmp_cell);
}

const Cell* Sheet::GetCell(Position pos) const {
	if (!pos.IsValid()) throw InvalidPositionException("Invalid position");

	try {
		return sheet_.at(pos.row).at(pos.col).get();
	}
	catch (const std::out_of_range& e) {
		return nullptr;
	}

	//auto rows_it = sheet_.find(pos.row);
	//if (rows_it != sheet_.end()) {
	//	auto cells_it = rows_it->second.find(pos.col);
	//	if (cells_it != rows_it->second.end()) {
	//		return cells_it->second.get();
	//	}
	//}
	//return nullptr;
}
Cell* Sheet::GetCell(Position pos) {
	if (!pos.IsValid()) throw InvalidPositionException("Invalid position");

	try {
		return sheet_.at(pos.row).at(pos.col).get();
	} 
	catch (const std::out_of_range& e) {
		return nullptr;
	}

	//auto rows_it = sheet_.find(pos.row);
	//if (rows_it != sheet_.end()) {
	//	auto cells_it = rows_it->second.find(pos.col);
	//	if (cells_it != rows_it->second.end()) {
	//		return cells_it->second.get();
	//	}
	//}
	//return nullptr;
}

void Sheet::ClearCell(Position pos) {
	if (!pos.IsValid()) throw InvalidPositionException("Invalid position");
	
	auto rows_it = sheet_.find(pos.row);
	if (rows_it != sheet_.end()) {
		rows_it->second.erase(pos.col);
	} else return;

	InvalidateCache(pos);
	graph_.EraseVertex(pos);
}

Size Sheet::GetPrintableSize() const {
	Size size{ 0, 0 };

	for (const auto& [row, col_map] : sheet_) {
		if (row >= size.rows && !col_map.empty()) size.rows = row + 1;
		for (const auto& [col, cell] : col_map) {
			if (col >= size.cols) size.cols = col + 1;
		}
	}

	return size;
}

void Sheet::PrintValues(std::ostream& output) const {
	Size size = GetPrintableSize();
	for (int row = 0; row < size.rows; row++) {
		bool is_first = true;
		for (int col = 0; col < size.cols; col++) {
			Position temp_pos{ row,col };
			const CellInterface* cell = GetCell(std::move(temp_pos));

			if (is_first) is_first = false;
			else output << "\t";

			if (cell != nullptr) {
				const auto value = cell->GetValue();
				if (auto error_ptr = std::get_if<FormulaError>(&value); error_ptr != nullptr) {
					output << *error_ptr;
				}
				else if (auto num_ptr = std::get_if<double>(&value); num_ptr != nullptr) {
					output << *num_ptr;
				}
				else {
					output << *std::get_if<std::string>(&value);
				}
			}
		}
		output << "\n";
	}
}

void Sheet::PrintTexts(std::ostream& output) const {
	Size size = GetPrintableSize();
	for (int row = 0; row < size.rows; row++) {
		bool is_first = true;
		for (int col = 0; col < size.cols; col++) {
			const CellInterface* cell = GetCell({ row,col });

			if (is_first) is_first = false;
			else output << "\t";

			if (cell != nullptr) {
				output << cell->GetText();
			}
		}
		output << "\n";
	}
}

const graph::DependencyGraph& Sheet::GetGraph() const {
	return graph_;
}

void Sheet::InvalidateCache(const Position& pos) {
	graph_.Traversal(
		pos,
		[&](const graph::Edge* edge) -> bool {
			Cell* cell = GetCell(edge->to);
			assert(cell != nullptr);

			cell->ClearCache();
			return false;  /// Continue traversal
		},
		graph::DependencyGraph::Direction::backward);
}

std::unique_ptr<SheetInterface> CreateSheet() {
	return std::make_unique<Sheet>();
}
