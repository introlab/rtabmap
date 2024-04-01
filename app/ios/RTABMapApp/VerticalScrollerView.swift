/*
 * Copyright (c) 2017 Razeware LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Notwithstanding the foregoing, you may not use, copy, modify, merge, publish,
 * distribute, sublicense, create a derivative work, and/or sell copies of the
 * Software in any work that is designed, intended, or marketed for pedagogical or
 * instructional purposes related to programming, coding, application development,
 * or information technology.  Permission for such use, copying, modification,
 * merger, publication, distribution, sublicensing, creation of derivative works,
 * or sale is expressly withheld.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

import UIKit

protocol VerticalViewDataSource: class {
    // Ask the data source how many views it wants to present inside the horizontal scroller
    func numberOfViews(in verticalScrollerView: VerticalScrollerView) -> Int
    // Ask the data source to return the view that should appear at <index>
    func getScrollerViewItem(_ verticalScrollerView: VerticalScrollerView, viewAt index: Int) -> UIView
}

protocol VerticalScrollerViewDelegate: class {
    // inform the delegate that the view at <index> has been selected
    func verticalScrollerView(_ verticalScrollerView: VerticalScrollerView, didSelectViewAt index: Int)
}

class VerticalScrollerView: UIView {
    weak var dataSource: VerticalViewDataSource?
    weak var delegate: VerticalScrollerViewDelegate?
    
    // 1
    private enum ViewConstants {
        static let Padding: CGFloat = 5
        static let Dimensions: CGFloat = 185
        static let Offset: CGFloat = 10
    }
    
    // 2
    private let scroller = UIScrollView()
    
    // 3
    private var contentViews = [UIView]()
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        initializeScrollView()
    }
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        initializeScrollView()
    }
    
    func initializeScrollView() {
        
        addSubview(scroller)
        
        scroller.translatesAutoresizingMaskIntoConstraints = false
        
        NSLayoutConstraint.activate([
            scroller.leadingAnchor.constraint(equalTo: self.leadingAnchor),
            scroller.trailingAnchor.constraint(equalTo: self.trailingAnchor),
            scroller.topAnchor.constraint(equalTo: self.topAnchor),
            scroller.bottomAnchor.constraint(equalTo: self.bottomAnchor)
        ])
        
        let tapRecognizer = UITapGestureRecognizer(target: self, action: #selector(scrollerTapped(gesture:)))
        scroller.addGestureRecognizer(tapRecognizer)
    }
    
    func scrollToView(at index: Int, animated: Bool = true) {
      let centralView = contentViews[index]
      let targetCenter = centralView.center
      let targetOffsetY = targetCenter.y - (scroller.bounds.height / 2)
      scroller.setContentOffset(CGPoint(x: 0, y: targetOffsetY), animated: animated)
    }
    
    @objc func scrollerTapped(gesture: UITapGestureRecognizer) {
        let location = gesture.location(in: scroller)
        guard
            let index = contentViews.index(where: { $0.frame.contains(location)})
        else { return }
        
        delegate?.verticalScrollerView(self, didSelectViewAt: index)
        scrollToView(at: index)
    }
    
    func view(at index :Int) -> UIView {
        return contentViews[index]
    }
    
    func reload() {
        // 1 - Check if there is a data source, if not there is nothing to load.
        guard let dataSource = dataSource else {
            return
        }
        
        //2 - Remove the old content views
        contentViews.forEach { $0.removeFromSuperview() }

        // 3 - yValue is the starting point of each view inside the scroller
        var yValue = ViewConstants.Offset
        // 4 - Fetch and add the new views
        contentViews = (0..<dataSource.numberOfViews(in: self)).map {
            index in
            // 5 - add a view at the right position
            yValue += ViewConstants.Padding
            let view = dataSource.getScrollerViewItem(self, viewAt: index)
            view.frame = CGRect(x: ViewConstants.Padding, y: CGFloat(yValue), width: ViewConstants.Dimensions*2, height: ViewConstants.Dimensions)
            scroller.addSubview(view)
            yValue += ViewConstants.Dimensions + ViewConstants.Padding
            // 6 - Store the view so we can reference it later
            return view
        }

        // 7
        scroller.contentSize = CGSize(width: frame.size.width, height: CGFloat(yValue + ViewConstants.Offset))
    }
}

