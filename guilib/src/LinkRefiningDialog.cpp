/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/gui/LinkRefiningDialog.h"
#include "ui_linkRefiningDialog.h"

#include <QPushButton>

namespace rtabmap {

LinkRefiningDialog::LinkRefiningDialog(QWidget * parent) :
	QDialog(parent),
	defaultNodeIdMin_(0),
	defaultNodeIdMax_(0),
	defaultMapIdMin_(0),
	defaultMapIdMax_(0)
{
	ui_ = new Ui_linkRefiningDialog();
	ui_->setupUi(this);

	ui_->comboBox_link_type->addItem("All");
	for(int i =0; i<Link::kPosePrior; ++i)
	{
		ui_->comboBox_link_type->addItem(Link::typeName((Link::Type)i).c_str());
		if(((Link::Type)i) == Link::kVirtualClosure)
		{
			ui_->comboBox_link_type->setItemData(i+1, 0, Qt::UserRole - 1);
		}
	}

	restoreDefaults();

	connect(ui_->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));
	connect(ui_->comboBox_link_type, SIGNAL(currentIndexChanged(int)), this, SLOT(updateIntraInterState()));
	connect(ui_->spinBox_node_from, SIGNAL(valueChanged(int)), this, SLOT(setRangeToNodeId()));
	connect(ui_->spinBox_node_to, SIGNAL(valueChanged(int)), this, SLOT(setRangeToNodeId()));
	connect(ui_->spinBox_map_from, SIGNAL(valueChanged(int)), this, SLOT(setRangeToMapId()));
	connect(ui_->spinBox_map_to, SIGNAL(valueChanged(int)), this, SLOT(setRangeToMapId()));
}

LinkRefiningDialog::~LinkRefiningDialog()
{
	delete ui_;
}

void LinkRefiningDialog::setMinMax(
		int nodeIdMin,
		int nodeIdMax,
		int mapIdMin,
		int mapIdMax)
{
	bool reset = defaultNodeIdMin_ == 0;
	defaultNodeIdMin_ = nodeIdMin;
	defaultNodeIdMax_ = nodeIdMax;
	defaultMapIdMin_ = mapIdMin;
	defaultMapIdMax_ = mapIdMax;
	ui_->spinBox_node_from->setMinimum(defaultNodeIdMin_);
	ui_->spinBox_node_to->setMaximum(defaultNodeIdMax_);
	ui_->spinBox_map_from->setMinimum(defaultMapIdMin_);
	ui_->spinBox_map_to->setMaximum(defaultMapIdMax_);
	if(reset)
	{
		restoreDefaults();
	}
}

Link::Type LinkRefiningDialog::getLinkType() const
{
	if(ui_->comboBox_link_type->currentIndex() == 0)
	{
		return Link::kEnd;
	}
	return (Link::Type)(ui_->comboBox_link_type->currentIndex()-1);
}

void LinkRefiningDialog::getIntraInterSessions(bool & intra, bool & inter) const
{
	intra = ui_->comboBox_link_inter_intra->currentIndex() == 0 || ui_->comboBox_link_inter_intra->currentIndex() == 1 || getLinkType() == Link::kNeighbor;
	inter = ui_->comboBox_link_inter_intra->currentIndex() == 0 || ui_->comboBox_link_inter_intra->currentIndex() == 2;
}

bool LinkRefiningDialog::isRangeByNodeId() const
{
	return ui_->radioButton_nodes->isChecked();
}

bool LinkRefiningDialog::isRangeByMapId() const
{
	return ui_->radioButton_maps->isChecked();
}

void LinkRefiningDialog::getRangeNodeId(int & from, int & to) const
{
	from = ui_->spinBox_node_from->value();
	to = ui_->spinBox_node_to->value();
}

void LinkRefiningDialog::getRangeMapId(int & from, int & to) const
{
	from = ui_->spinBox_map_from->value();
	to = ui_->spinBox_map_to->value();
}

void LinkRefiningDialog::restoreDefaults()
{
	ui_->comboBox_link_type->setCurrentIndex(0);
	ui_->comboBox_link_inter_intra->setCurrentIndex(0);
	ui_->spinBox_node_from->setValue(defaultNodeIdMin_);
	ui_->spinBox_node_to->setValue(defaultNodeIdMax_);
	ui_->spinBox_map_from->setValue(defaultMapIdMin_);
	ui_->spinBox_map_to->setValue(defaultMapIdMax_);

	ui_->radioButton_nodes->setChecked(true);
}

void LinkRefiningDialog::updateIntraInterState()
{
	ui_->comboBox_link_inter_intra->setEnabled(getLinkType() != Link::kNeighbor);
}

void LinkRefiningDialog::setRangeToNodeId()
{
	ui_->radioButton_nodes->setChecked(true);
}

void LinkRefiningDialog::setRangeToMapId()
{
	ui_->radioButton_maps->setChecked(true);
}


}
