// Benchmark "adder" written by ABC on Thu Jul 11 11:20:49 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n326,
    new_n328, new_n330, new_n331;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[2] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[1] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  160nm_fiao0012aa1n02p5x5     g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  oabi12aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n123));
  xorc02aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n125), .b(new_n98), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n126), .clkout(new_n134));
  aob012aa1n02x5               g039(.a(new_n134), .b(new_n97), .c(new_n127), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n133), .b(new_n135), .c(new_n130), .d(new_n128), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n135), .b(new_n133), .c(new_n130), .d(new_n128), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  nona22aa1n02x4               g046(.a(new_n136), .b(new_n141), .c(new_n131), .out0(new_n142));
  nanb02aa1n02x5               g047(.a(new_n139), .b(new_n140), .out0(new_n143));
  oaoi13aa1n02x5               g048(.a(new_n143), .b(new_n136), .c(\a[11] ), .d(\b[10] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n142), .b(new_n144), .out0(\s[12] ));
  nanp02aa1n02x5               g050(.a(new_n109), .b(new_n117), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n123), .clkout(new_n147));
  nano23aa1n02x4               g052(.a(new_n131), .b(new_n139), .c(new_n140), .d(new_n132), .out0(new_n148));
  nanp03aa1n02x5               g053(.a(new_n148), .b(new_n124), .c(new_n128), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n139), .b(new_n131), .c(new_n140), .o1(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n148), .c(new_n135), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n149), .c(new_n146), .d(new_n147), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  norp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nano23aa1n02x4               g069(.a(new_n155), .b(new_n163), .c(new_n164), .d(new_n156), .out0(new_n165));
  aoi012aa1n02x5               g070(.a(new_n163), .b(new_n155), .c(new_n164), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n162), .b(new_n167), .c(new_n153), .d(new_n165), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n162), .b(new_n167), .c(new_n153), .d(new_n165), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  nona22aa1n02x4               g076(.a(new_n168), .b(new_n171), .c(new_n159), .out0(new_n172));
  xnrc02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .out0(new_n173));
  oaoi13aa1n02x5               g078(.a(new_n173), .b(new_n168), .c(\a[15] ), .d(\b[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n172), .b(new_n174), .out0(\s[16] ));
  nanp03aa1n02x5               g080(.a(new_n165), .b(new_n162), .c(new_n171), .o1(new_n176));
  norp02aa1n02x5               g081(.a(new_n176), .b(new_n149), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n178));
  norp02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  nona22aa1n02x4               g085(.a(new_n171), .b(new_n166), .c(new_n161), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  nanp03aa1n02x5               g088(.a(new_n135), .b(new_n133), .c(new_n141), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n176), .b(new_n184), .c(new_n150), .o1(new_n185));
  nano32aa1n02x4               g090(.a(new_n185), .b(new_n181), .c(new_n183), .d(new_n180), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n178), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n197));
  norp02aa1n02x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n195), .c(new_n178), .d(new_n186), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n202), .b(new_n207), .c(new_n199), .d(new_n204), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n203), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nona23aa1n02x4               g115(.a(new_n206), .b(new_n203), .c(new_n202), .d(new_n205), .out0(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  nanp02aa1n02x5               g117(.a(new_n194), .b(new_n212), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n206), .b(new_n205), .c(new_n202), .o1(new_n214));
  oai012aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n198), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n213), .c(new_n178), .d(new_n186), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xorc02aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n219), .b(new_n221), .c(new_n217), .d(new_n220), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[21] ), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[22] ), .clkout(new_n226));
  xroi22aa1d04x5               g131(.a(new_n225), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  nanp03aa1n02x5               g132(.a(new_n227), .b(new_n194), .c(new_n212), .o1(new_n228));
  oai112aa1n02x5               g133(.a(new_n204), .b(new_n207), .c(new_n197), .d(new_n196), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n227), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oao003aa1n02x5               g136(.a(new_n226), .b(new_n231), .c(new_n219), .carry(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n230), .c(new_n229), .d(new_n214), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n228), .c(new_n178), .d(new_n186), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n236), .d(new_n239), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  and002aa1n02x5               g148(.a(new_n240), .b(new_n239), .o(new_n244));
  nanb03aa1n02x5               g149(.a(new_n213), .b(new_n244), .c(new_n227), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(\a[24] ), .clkout(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\b[23] ), .clkout(new_n247));
  oao003aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n238), .carry(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n234), .c(new_n244), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n178), .d(new_n186), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  oao003aa1n02x5               g162(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n258));
  nano23aa1n02x4               g163(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n259));
  aobi12aa1n02x5               g164(.a(new_n108), .b(new_n259), .c(new_n258), .out0(new_n260));
  nano23aa1n02x4               g165(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n115), .c(new_n116), .out0(new_n262));
  oai012aa1n02x5               g167(.a(new_n147), .b(new_n260), .c(new_n262), .o1(new_n263));
  norp03aa1n02x5               g168(.a(new_n173), .b(new_n166), .c(new_n161), .o1(new_n264));
  aoi012aa1n02x5               g169(.a(new_n126), .b(new_n97), .c(new_n127), .o1(new_n265));
  norb03aa1n02x5               g170(.a(new_n133), .b(new_n143), .c(new_n265), .out0(new_n266));
  nona23aa1n02x4               g171(.a(new_n164), .b(new_n156), .c(new_n155), .d(new_n163), .out0(new_n267));
  norp03aa1n02x5               g172(.a(new_n267), .b(new_n173), .c(new_n161), .o1(new_n268));
  oai012aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n151), .o1(new_n269));
  nona32aa1n02x4               g174(.a(new_n269), .b(new_n182), .c(new_n264), .d(new_n179), .out0(new_n270));
  and002aa1n02x5               g175(.a(new_n254), .b(new_n253), .o(new_n271));
  nano22aa1n02x4               g176(.a(new_n228), .b(new_n244), .c(new_n271), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n270), .c(new_n263), .d(new_n177), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n271), .b(new_n248), .c(new_n234), .d(new_n244), .o1(new_n274));
  oai022aa1n02x5               g179(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n275));
  aob012aa1n02x5               g180(.a(new_n275), .b(\b[25] ), .c(\a[26] ), .out0(new_n276));
  xorc02aa1n02x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aoi013aa1n02x4               g183(.a(new_n278), .b(new_n273), .c(new_n274), .d(new_n276), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n272), .b(new_n178), .c(new_n186), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n244), .b(new_n232), .c(new_n215), .d(new_n227), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n248), .clkout(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n271), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n276), .b(new_n283), .c(new_n281), .d(new_n282), .o1(new_n284));
  norp03aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n277), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n279), .b(new_n285), .o1(\s[27] ));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n287), .clkout(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n279), .b(new_n288), .c(new_n289), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n277), .b(new_n284), .c(new_n280), .o1(new_n291));
  aoi012aa1n02x5               g196(.a(new_n289), .b(new_n291), .c(new_n288), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n290), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n277), .b(new_n289), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n284), .c(new_n280), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n294), .clkout(new_n299));
  aoi013aa1n02x4               g204(.a(new_n299), .b(new_n273), .c(new_n274), .d(new_n276), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n296), .c(new_n297), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n298), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g208(.a(new_n277), .b(new_n297), .c(new_n289), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(new_n284), .c(new_n280), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n304), .clkout(new_n309));
  aoi013aa1n02x4               g214(.a(new_n309), .b(new_n273), .c(new_n274), .d(new_n276), .o1(new_n310));
  nano22aa1n02x4               g215(.a(new_n310), .b(new_n306), .c(new_n307), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n308), .b(new_n311), .o1(\s[30] ));
  norb02aa1n02x5               g217(.a(new_n304), .b(new_n307), .out0(new_n313));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n313), .clkout(new_n314));
  aoi013aa1n02x4               g219(.a(new_n314), .b(new_n273), .c(new_n274), .d(new_n276), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  nano22aa1n02x4               g222(.a(new_n315), .b(new_n316), .c(new_n317), .out0(new_n318));
  oai012aa1n02x5               g223(.a(new_n313), .b(new_n284), .c(new_n280), .o1(new_n319));
  aoi012aa1n02x5               g224(.a(new_n317), .b(new_n319), .c(new_n316), .o1(new_n320));
  norp02aa1n02x5               g225(.a(new_n320), .b(new_n318), .o1(\s[31] ));
  xnrb03aa1n02x5               g226(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g227(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g230(.a(\a[5] ), .b(\b[4] ), .c(new_n260), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g232(.a(new_n118), .b(new_n119), .c(new_n326), .carry(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g234(.a(new_n118), .b(new_n119), .c(new_n326), .o1(new_n330));
  oaoi03aa1n02x5               g235(.a(\a[7] ), .b(\b[6] ), .c(new_n330), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n124), .b(new_n146), .c(new_n147), .out0(\s[9] ));
endmodule


