/*    Copyright (c) 2010-2019, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 *
 *    References
 *      E. B. Saff, et al. "Distributing many points on a sphere".
 *          The Mathematical Intelligencer 19. 1(1997): 5–11.
 *      Frank G. Lemoine, et al. "High‒degree gravity models from GRAIL primary mission data".
 *          Journal of Geophysical Research: Planets 118. 8(2013): 1676–1698.
 */

#ifndef TUDAT_RADIATIONSOURCEMODEL_H
#define TUDAT_RADIATIONSOURCEMODEL_H

#include <vector>
#include <map>
#include <utility>
#include <functional>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tudat/astro/electromagnetism/luminosityModel.h"
#include "tudat/astro/electromagnetism/reflectionLaw.h"
#include "tudat/astro/basic_astro/bodyShapeModel.h"

namespace tudat
{
namespace electromagnetism
{

typedef std::vector<std::pair<double, Eigen::Vector3d>> IrradianceWithSourceList;


class RadiationSourceModel
{
public:
    explicit RadiationSourceModel() = default;

    virtual ~RadiationSourceModel() = default;

    void updateMembers(double currentTime );

    /*!
     * Evaluate the irradiance [W/m²] at a certain position due to this source.
     * @param targetPosition Position where to evaluate the irradiance in local (i.e. source-fixed) coordinates
     * @param originalSourceIrradiance Irradiance from the original source (if applicable)
     * @param originalSourceToSourceDirection Direction of incoming radiation in local (i.e. source-fixed) coordinates
     * @return A list of irradiances and their source-fixed origin. Every element can be thought of as a ray.
     * Single element for point sources, multiple elements for paneled sources.
     */
    virtual IrradianceWithSourceList evaluateIrradianceAtPosition(
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const = 0;

protected:
    virtual void updateMembers_(const double currentTime) {};

    double currentTime_{TUDAT_NAN};
};

//*********************************************************************************************
//   Isotropic point radiation source
//*********************************************************************************************

class IsotropicPointRadiationSourceModel : public RadiationSourceModel
{
public:
    explicit IsotropicPointRadiationSourceModel(
            const std::shared_ptr<LuminosityModel>& luminosityModel) :
        luminosityModel_(luminosityModel) {}

    IrradianceWithSourceList evaluateIrradianceAtPosition(
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const override;

    double evaluateIrradianceAtPosition(const Eigen::Vector3d& targetPosition) const;

    std::shared_ptr<LuminosityModel> getLuminosityModel() const
    {
        return luminosityModel_;
    }

private:
    void updateMembers_(double currentTime) override;

    std::shared_ptr<LuminosityModel> luminosityModel_;
};

//*********************************************************************************************
//   Paneled radiation source
//*********************************************************************************************

class PaneledRadiationSourceModel : public RadiationSourceModel
{
public:
    class Panel;
    class PanelRadiosityModel; // cannot make this nested class of Panel since forward declaration is impossible

    explicit PaneledRadiationSourceModel(
            const std::shared_ptr<basic_astrodynamics::BodyShapeModel>& sourceBodyShapeModel) :
        sourceBodyShapeModel_(sourceBodyShapeModel) {}

    explicit PaneledRadiationSourceModel() :
            PaneledRadiationSourceModel(nullptr) {}

    IrradianceWithSourceList evaluateIrradianceAtPosition(
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const override;

    virtual const std::vector<Panel>& getPanels() const = 0;

protected:
    void updateMembers_(double currentTime) override;

    std::shared_ptr<basic_astrodynamics::BodyShapeModel> sourceBodyShapeModel_;
};

class StaticallyPaneledRadiationSourceModel : public PaneledRadiationSourceModel
{
public:
    explicit StaticallyPaneledRadiationSourceModel(
            const std::vector<Panel>& panels) :
            PaneledRadiationSourceModel(),
            n_(panels.size()),
            panels_(panels) {}

    explicit StaticallyPaneledRadiationSourceModel(
            const std::shared_ptr<basic_astrodynamics::BodyShapeModel>& sourceBodyShapeModel,
            const std::function<std::vector<std::shared_ptr<PanelRadiosityModel>>(double, double)>& radiosityModelFunction,
            int n) :
            PaneledRadiationSourceModel(sourceBodyShapeModel),
            n_(n),
            radiosityModelFunction_(radiosityModelFunction) {}

    explicit StaticallyPaneledRadiationSourceModel(
            const std::shared_ptr<basic_astrodynamics::BodyShapeModel>& sourceBodyShapeModel,
            const std::vector<std::shared_ptr<PanelRadiosityModel>>& radiosityModel,
            int n) :
            StaticallyPaneledRadiationSourceModel(sourceBodyShapeModel, [=](double, double) { return radiosityModel; }, n) {}

    const std::vector<Panel>& getPanels() const override
    {
        return panels_;
    }

private:
    void updateMembers_(double currentTime) override;

    unsigned int n_;
    std::function<std::vector<std::shared_ptr<PanelRadiosityModel>>(double, double)> radiosityModelFunction_;
    std::vector<Panel> panels_;
};

//class DynamicallyPaneledRadiationSourceModel : public PaneledRadiationSourceModel
//{
//private:
//    // keep target-specific panel lists, otherwise may have to regenerate a lot
//    std::map<std::string, std::vector<Panel>> panels_;
//};

class PaneledRadiationSourceModel::Panel
{
public:
    explicit Panel(
            double area,
            const Eigen::Vector3d& relativeCenter,
            const Eigen::Vector3d& surfaceNormal,
            const std::vector<std::shared_ptr<PanelRadiosityModel>>& radiosityModels) :
        area_(area),
        relativeCenter_(relativeCenter),
        surfaceNormal_(surfaceNormal),
        radiosityModels_(radiosityModels) {}

    double getArea() const
    {
        return area_;
    }

    const Eigen::Vector3d& getRelativeCenter() const
    {
        return relativeCenter_;
    }

    const Eigen::Vector3d& getSurfaceNormal() const
    {
        return surfaceNormal_;
    }

    const std::vector<std::shared_ptr<PanelRadiosityModel>>& getRadiosityModels() const
    {
        return radiosityModels_;
    }

private:
    double area_;
    Eigen::Vector3d relativeCenter_;
    Eigen::Vector3d surfaceNormal_;
    std::vector<std::shared_ptr<PanelRadiosityModel>> radiosityModels_;
};

class PaneledRadiationSourceModel::PanelRadiosityModel
{
public:
    virtual ~PanelRadiosityModel() = default;

    /*!
     * Evaluate the irradiance [W/m²] at a certain position due to this panel.
     * @param panel The panel this radiosity model belongs to
     * @param targetPosition Position where to evaluate the irradiance in local coordinates (source rotation, centered in panel)
     * @return The irradiance
     */
    virtual double evaluateIrradianceAtPosition(
            const Panel& panel,
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const = 0;
};

class AlbedoPanelRadiosityModel : public PaneledRadiationSourceModel::PanelRadiosityModel
{
public:
    explicit AlbedoPanelRadiosityModel(
            const std::shared_ptr<ReflectionLaw>& reflectionLaw) :
            reflectionLaw_(reflectionLaw) {}

    double evaluateIrradianceAtPosition(
            const PaneledRadiationSourceModel::Panel& panel,
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const override;

private:
    std::shared_ptr<ReflectionLaw> reflectionLaw_;
};

/*!
 * Panel radiosity model for thermal emissions, based on angle to subsolar point. This model was introduced in
 * Lemoine (2013) for lunar thermal radiation.
 */
class AngleBasedThermalPanelRadiosityModel : public PaneledRadiationSourceModel::PanelRadiosityModel
{
public:
    explicit AngleBasedThermalPanelRadiosityModel(
            double minTemperature,
            double maxTemperature,
            double emissivity) :
            minTemperature_(minTemperature),
            maxTemperature_(maxTemperature),
            emissivity_(emissivity) {}

    double evaluateIrradianceAtPosition(
            const PaneledRadiationSourceModel::Panel& panel,
            const Eigen::Vector3d& targetPosition,
            double originalSourceIrradiance,
            const Eigen::Vector3d& originalSourceToSourceDirection) const override;

private:
    double minTemperature_;
    double maxTemperature_;
    double emissivity_;
};

/*!
 * Generate evenly spaced points on a sphere using algorithm from Saff (1997).
 * @param n number of points to generate
 * @return a pair of vectors, first vector are polar angles, second vector are azimuth angles
 */
std::pair<std::vector<double>, std::vector<double>> generateEvenlySpacedPoints(unsigned int n);


} // tudat
} // electromagnetism

#endif //TUDAT_RADIATIONSOURCEMODEL_H
