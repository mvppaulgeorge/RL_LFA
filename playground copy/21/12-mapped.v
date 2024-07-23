// Benchmark "adder" written by ABC on Thu Jul 11 12:25:45 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n324, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n02x4               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  norp02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  norp03aa1n02x5               g010(.a(new_n101), .b(new_n104), .c(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .out0(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .out0(new_n112));
  oai022aa1n02x5               g017(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n113));
  aob012aa1n02x5               g018(.a(new_n113), .b(\b[3] ), .c(\a[4] ), .out0(new_n114));
  oai013aa1n02x4               g019(.a(new_n114), .b(new_n111), .c(new_n112), .d(new_n110), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(new_n97), .clkout(new_n116));
  nanp02aa1n02x5               g021(.a(new_n99), .b(new_n98), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[5] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[4] ), .clkout(new_n119));
  aoi013aa1n02x4               g024(.a(new_n102), .b(new_n103), .c(new_n118), .d(new_n119), .o1(new_n120));
  oai112aa1n02x5               g025(.a(new_n116), .b(new_n117), .c(new_n101), .d(new_n120), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n115), .c(new_n106), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  norp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nona23aa1n02x4               g036(.a(new_n131), .b(new_n129), .c(new_n128), .d(new_n130), .out0(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(new_n132), .clkout(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n121), .c(new_n115), .d(new_n106), .o1(new_n134));
  aoi012aa1n02x5               g039(.a(new_n130), .b(new_n128), .c(new_n131), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n127), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n125), .clkout(new_n137));
  nanp02aa1n02x5               g042(.a(new_n115), .b(new_n106), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n121), .clkout(new_n139));
  nanp02aa1n02x5               g044(.a(new_n138), .b(new_n139), .o1(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n135), .clkout(new_n141));
  aoai13aa1n02x5               g046(.a(new_n127), .b(new_n141), .c(new_n140), .d(new_n133), .o1(new_n142));
  norp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n137), .out0(\s[12] ));
  nona23aa1n02x4               g051(.a(new_n144), .b(new_n126), .c(new_n125), .d(new_n143), .out0(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  oai012aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n135), .o1(new_n150));
  oabi12aa1n02x5               g055(.a(new_n150), .b(new_n134), .c(new_n147), .out0(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .o(new_n153));
  nano23aa1n02x4               g058(.a(new_n125), .b(new_n143), .c(new_n144), .d(new_n126), .out0(new_n154));
  nano22aa1n02x4               g059(.a(new_n122), .b(new_n133), .c(new_n154), .out0(new_n155));
  xnrc02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  oabi12aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n150), .out0(new_n157));
  xnrc02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  xobna2aa1n03x5               g063(.a(new_n158), .b(new_n157), .c(new_n153), .out0(\s[14] ));
  norp02aa1n02x5               g064(.a(new_n158), .b(new_n156), .o1(new_n160));
  nano32aa1n02x4               g065(.a(new_n122), .b(new_n160), .c(new_n133), .d(new_n154), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n160), .b(new_n148), .c(new_n154), .d(new_n141), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  nanp02aa1n02x5               g069(.a(new_n162), .b(new_n164), .o1(new_n165));
  norp02aa1n02x5               g070(.a(new_n161), .b(new_n165), .o1(new_n166));
  xnrc02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  xnrc02aa1n02x5               g073(.a(new_n166), .b(new_n168), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g074(.clk(\a[16] ), .clkout(new_n170));
  norp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n171), .b(new_n168), .c(new_n161), .d(new_n165), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[15] ), .c(new_n170), .out0(\s[16] ));
  nanb02aa1n02x5               g078(.a(new_n128), .b(new_n129), .out0(new_n174));
  nanb02aa1n02x5               g079(.a(new_n130), .b(new_n131), .out0(new_n175));
  nona22aa1n02x4               g080(.a(new_n154), .b(new_n175), .c(new_n174), .out0(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .out0(new_n177));
  norp02aa1n02x5               g082(.a(new_n177), .b(new_n167), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n176), .b(new_n160), .c(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n121), .c(new_n115), .d(new_n106), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n178), .b(new_n163), .c(new_n150), .d(new_n160), .o1(new_n181));
  oai022aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n182));
  oaib12aa1n02x5               g087(.a(new_n182), .b(new_n170), .c(\b[15] ), .out0(new_n183));
  nanp03aa1n02x5               g088(.a(new_n180), .b(new_n181), .c(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n186), .b(new_n184), .c(new_n187), .o1(new_n188));
  xnrb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aobi12aa1n02x5               g094(.a(new_n183), .b(new_n165), .c(new_n178), .out0(new_n190));
  norp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nano23aa1n02x4               g097(.a(new_n186), .b(new_n191), .c(new_n192), .d(new_n187), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  aoi012aa1n02x5               g099(.a(new_n191), .b(new_n186), .c(new_n192), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n190), .d(new_n180), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n199), .b(new_n204), .c(new_n196), .d(new_n201), .o1(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n199), .clkout(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n195), .clkout(new_n207));
  aoai13aa1n02x5               g112(.a(new_n201), .b(new_n207), .c(new_n184), .d(new_n193), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n204), .clkout(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n206), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n205), .o1(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n199), .b(new_n202), .c(new_n203), .d(new_n200), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n193), .o1(new_n213));
  nona23aa1n02x4               g118(.a(new_n203), .b(new_n200), .c(new_n199), .d(new_n202), .out0(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[20] ), .b(\b[19] ), .c(new_n206), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n195), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n213), .c(new_n190), .d(new_n180), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n221), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n213), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n222), .b(new_n217), .c(new_n184), .d(new_n226), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n223), .clkout(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n227), .c(new_n225), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n224), .o1(\s[22] ));
  nano22aa1n02x4               g135(.a(new_n213), .b(new_n222), .c(new_n223), .out0(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[21] ), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[22] ), .clkout(new_n234));
  xroi22aa1d04x5               g139(.a(new_n233), .b(\b[20] ), .c(new_n234), .d(\b[21] ), .out0(new_n235));
  oaoi03aa1n02x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n217), .c(new_n235), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n232), .c(new_n190), .d(new_n180), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n240), .clkout(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n237), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n241), .b(new_n245), .c(new_n184), .d(new_n231), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n242), .clkout(new_n247));
  aoi012aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n244), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n248), .b(new_n243), .o1(\s[24] ));
  and002aa1n02x5               g154(.a(new_n242), .b(new_n241), .o(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  nano32aa1n02x4               g156(.a(new_n251), .b(new_n235), .c(new_n212), .d(new_n193), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n235), .b(new_n215), .c(new_n212), .d(new_n207), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n236), .clkout(new_n255));
  nanp02aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  oai022aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n257), .b(new_n256), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n251), .c(new_n254), .d(new_n255), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n253), .c(new_n190), .d(new_n180), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n263), .clkout(new_n267));
  aoai13aa1n02x5               g172(.a(new_n264), .b(new_n259), .c(new_n184), .d(new_n252), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n265), .clkout(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n268), .c(new_n267), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n266), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n178), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n183), .b(new_n272), .c(new_n162), .d(new_n164), .o1(new_n273));
  and002aa1n02x5               g178(.a(new_n265), .b(new_n264), .o(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n274), .clkout(new_n275));
  nano23aa1n02x4               g180(.a(new_n213), .b(new_n275), .c(new_n250), .d(new_n235), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n273), .c(new_n140), .d(new_n179), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n278));
  aobi12aa1n02x5               g183(.a(new_n278), .b(new_n259), .c(new_n274), .out0(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  aobi12aa1n02x5               g188(.a(new_n280), .b(new_n277), .c(new_n279), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n250), .b(new_n236), .c(new_n217), .d(new_n235), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n278), .b(new_n275), .c(new_n287), .d(new_n258), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n280), .b(new_n288), .c(new_n184), .d(new_n276), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n285), .b(new_n289), .c(new_n283), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g196(.a(new_n280), .b(new_n285), .out0(new_n292));
  aobi12aa1n02x5               g197(.a(new_n292), .b(new_n277), .c(new_n279), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n292), .b(new_n288), .c(new_n184), .d(new_n276), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n280), .b(new_n295), .c(new_n285), .out0(new_n301));
  aobi12aa1n02x5               g206(.a(new_n301), .b(new_n277), .c(new_n279), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n301), .b(new_n288), .c(new_n184), .d(new_n276), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n305), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n301), .b(new_n304), .out0(new_n309));
  aobi12aa1n02x5               g214(.a(new_n309), .b(new_n277), .c(new_n279), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n309), .b(new_n288), .c(new_n184), .d(new_n276), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g225(.a(new_n118), .b(new_n119), .c(new_n115), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g227(.a(new_n105), .b(new_n104), .o1(new_n323));
  aobi12aa1n02x5               g228(.a(new_n120), .b(new_n115), .c(new_n323), .out0(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(\a[7] ), .b(\b[6] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g232(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


