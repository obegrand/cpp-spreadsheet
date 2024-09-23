#pragma once

#include "graph.h"

class Sheet : public SheetInterface {
public:
	Sheet() = default;
	~Sheet() = default;

	void SetCell(Position pos, std::string text) override;

	const Cell* GetCell(Position pos) const override;
	Cell* GetCell(Position pos) override;

	void ClearCell(Position pos) override;

	Size GetPrintableSize() const override;

	void PrintValues(std::ostream& output) const override;
	void PrintTexts(std::ostream& output) const override;

	const graph::DependencyGraph& GetGraph() const;

private:
	void InvalidateCache(const Position& pos);

	//	sheet impl: row->col->cell
	std::unordered_map<int, std::unordered_map<int, std::unique_ptr<Cell>>> sheet_;
	graph::DependencyGraph graph_;
};