#pragma once

#include "common.h"
#include "formula.h"

#include <functional>
#include <unordered_set>

class Sheet;

class Cell : public CellInterface {
public:
	explicit Cell(SheetInterface& sheet);
	~Cell();

	void Set(std::string text);
	void Clear();

	CellInterface::Value GetValue() const override;
	std::string GetText() const override;
	std::vector<Position> GetReferencedCells() const override;

	void ClearCache();
	bool HasCache() const;

private:
	class Impl {
	public:
		virtual ~Impl() = default;
		virtual Value GetValue() const = 0;
		virtual std::string GetText() const = 0;
		virtual std::vector<Position> GetReferencedCells() const {
			return {};
		}
	};

	class EmptyImpl : public Impl {
	public:
		Value GetValue() const override {
			return 0.0;
		}
		std::string GetText() const override {
			return "";
		}
	};

	class TextImpl : public Impl {
	public:
		TextImpl(std::string text) : text_{ std::move(text) } {}
		Value GetValue() const override {
			if (text_.length() > 0 && text_[0] == '\'') return text_.substr(1);
			else return text_;
		}
		std::string GetText() const override {
			return text_;
		}
	private:
		std::string text_;
	};

	class FormulaImpl : public Impl {
	public:
		FormulaImpl(std::string text, SheetInterface& sheet) : formula_{ ParseFormula(std::move(text)) }, sheet_{ sheet } {}

		Value GetValue() const override {
			auto temp_value = formula_->Evaluate(sheet_);
			if (std::holds_alternative<double>(temp_value)) {
				return std::get<double>(temp_value);
			}
			return std::get<FormulaError>(temp_value);
		}

		std::string GetText() const override {
			return '=' + formula_->GetExpression();
		}

		std::vector<Position> GetReferencedCells() const override {
			return formula_->GetReferencedCells();
		}
	private:
		std::unique_ptr<FormulaInterface> formula_;
		const SheetInterface& sheet_;
	};

	std::unique_ptr<Impl> impl_;
	SheetInterface& sheet_;
	mutable std::unique_ptr<Value> cache_;
};