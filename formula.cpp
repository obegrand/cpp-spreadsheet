#include "formula.h"

#include "FormulaAST.h"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <sstream>

using namespace std::literals;

std::ostream& operator<<(std::ostream& output, FormulaError fe) {
	return output << "#ARITHM!";
}

namespace {
	class Formula : public FormulaInterface {
	public:
		explicit Formula(std::string expression) :
			ast_(ParseFormulaAST(std::move(expression))) {
		}

		Value Evaluate(const SheetInterface& sheet) const override {
			const auto cellfunc = [&sheet](const Position& position) -> double {
				const CellInterface* cell_ptr = sheet.GetCell(position);
				if (cell_ptr == nullptr) return 0.0;
				CellInterface::Value value = cell_ptr->GetValue();
				
				const FormulaError* error = std::get_if<FormulaError>(&value);
				if (error != nullptr) throw* error;

				const double* result = std::get_if<double>(&value);
				if (result != nullptr) return *result;

				const std::string* str = std::get_if<std::string>(&value);
				if (str != nullptr) {
					size_t idx = 0;
					try {
						double result = std::stod(*str, &idx);
						if (str->length() > idx) throw FormulaError(FormulaError::Category::Value);
						return result;
					} catch (...) {}
				}
				throw FormulaError(FormulaError::Category::Value);

				};

			try {
				return ast_.Execute(cellfunc);
			}
			catch (const FormulaError& error) {
				return error;
			}
		}

		std::string GetExpression() const override {
			std::ostringstream out;
			ast_.PrintFormula(out);
			return out.str();
		}

		std::vector<Position> GetReferencedCells() const override {
			const auto& cell_refs = ast_.GetCells();
			std::vector<Position> result = std::vector<Position>(cell_refs.begin(), cell_refs.end());

			std::sort(result.begin(), result.end());
			auto last = std::unique(result.begin(), result.end());
			result.erase(last, result.end());

			return result;
		}

	private:
		FormulaAST ast_;
	};
}  // namespace

std::unique_ptr<FormulaInterface> ParseFormula(std::string expression) {
	try {
		return std::make_unique<Formula>(expression);
	}
	catch (...) {
		throw FormulaException("Parsing formula expression failure"s);
	}
}

FormulaError::FormulaError(Category category) : category_(category) {}

FormulaError::Category FormulaError::GetCategory() const {
	return category_;
}

bool FormulaError::operator==(FormulaError rhs) const {
	return ToString() == rhs.ToString();
}

std::string_view FormulaError::ToString() const {
	switch (category_) {
	case Category::Value:
		return "#VALUE!"sv;
	case Category::Arithmetic:
		return "#ARITHM!"sv;
	case Category::Ref:
		return "#REF!"sv;
	default:
		return ""sv;
	}
}