// Benchmark "adder" written by ABC on Thu Jul 11 11:49:49 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  norp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  norp02aa1n02x5               g016(.a(new_n111), .b(new_n106), .o1(new_n112));
  and002aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o(new_n113));
  160nm_ficinv00aa1n08x5       g018(.clk(\a[3] ), .clkout(new_n114));
  160nm_ficinv00aa1n08x5       g019(.clk(\b[2] ), .clkout(new_n115));
  nanp02aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n116), .b(new_n117), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[0] ), .b(\a[1] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[1] ), .b(\a[2] ), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oa0022aa1n02x5               g027(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n123));
  oaoi13aa1n02x5               g028(.a(new_n113), .b(new_n123), .c(new_n122), .d(new_n118), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n108), .b(new_n109), .c(new_n107), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(new_n106), .c(new_n125), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n124), .d(new_n112), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oaoi03aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(new_n101), .o1(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  nano23aa1n02x4               g040(.a(new_n97), .b(new_n100), .c(new_n128), .d(new_n98), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n127), .c(new_n124), .d(new_n112), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n133), .b(new_n137), .c(new_n135), .out0(\s[11] ));
  nanb02aa1n02x5               g043(.a(new_n134), .b(new_n137), .out0(new_n139));
  aoi012aa1n02x5               g044(.a(new_n131), .b(new_n139), .c(new_n132), .o1(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[12] ), .b(\a[13] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[12] ), .b(\a[13] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nano23aa1n02x4               g051(.a(new_n131), .b(new_n145), .c(new_n146), .d(new_n132), .out0(new_n147));
  and002aa1n02x5               g052(.a(new_n147), .b(new_n136), .o(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n127), .c(new_n124), .d(new_n112), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n145), .b(new_n131), .c(new_n146), .o1(new_n150));
  aobi12aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n134), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n144), .b(new_n149), .c(new_n151), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n142), .clkout(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n144), .clkout(new_n154));
  aoai13aa1n02x5               g059(.a(new_n153), .b(new_n154), .c(new_n149), .d(new_n151), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n142), .b(new_n157), .c(new_n158), .d(new_n143), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  oaoi03aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n149), .d(new_n151), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  aoi122aa1n02x5               g074(.a(new_n165), .b(new_n169), .c(new_n168), .d(new_n163), .e(new_n166), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n167), .b(new_n169), .out0(new_n172));
  norp02aa1n02x5               g077(.a(new_n171), .b(new_n172), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n170), .o1(\s[16] ));
  aoi012aa1n02x5               g079(.a(new_n127), .b(new_n124), .c(new_n112), .o1(new_n175));
  nano23aa1n02x4               g080(.a(new_n165), .b(new_n167), .c(new_n169), .d(new_n166), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n159), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n177), .b(new_n136), .c(new_n147), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  nanp02aa1n02x5               g084(.a(new_n176), .b(new_n161), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n165), .b(new_n169), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n147), .b(new_n134), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n177), .b(new_n182), .c(new_n150), .o1(new_n183));
  nano32aa1n02x4               g088(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n168), .out0(new_n184));
  oai012aa1n02x5               g089(.a(new_n184), .b(new_n175), .c(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  aoai13aa1n02x5               g096(.a(new_n178), .b(new_n127), .c(new_n124), .d(new_n112), .o1(new_n192));
  xroi22aa1d04x5               g097(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  oai022aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  oaib12aa1n02x5               g100(.a(new_n195), .b(new_n187), .c(\b[17] ), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n194), .c(new_n184), .d(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  xorc02aa1n02x5               g105(.a(\a[19] ), .b(\b[18] ), .out0(new_n201));
  xnrc02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  aoi112aa1n02x5               g108(.a(new_n200), .b(new_n203), .c(new_n197), .d(new_n201), .o1(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n200), .clkout(new_n205));
  nanp02aa1n02x5               g110(.a(new_n197), .b(new_n201), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n202), .b(new_n206), .c(new_n205), .o1(new_n207));
  norp02aa1n02x5               g112(.a(new_n207), .b(new_n204), .o1(\s[20] ));
  xnrc02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .out0(new_n209));
  nona22aa1n02x4               g114(.a(new_n193), .b(new_n209), .c(new_n202), .out0(new_n210));
  oao003aa1n02x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .carry(new_n211));
  oai013aa1n02x4               g116(.a(new_n211), .b(new_n209), .c(new_n202), .d(new_n196), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(new_n184), .d(new_n192), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  norp02aa1n02x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n216), .b(new_n221), .c(new_n214), .d(new_n218), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n216), .clkout(new_n223));
  nanp02aa1n02x5               g128(.a(new_n214), .b(new_n218), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n221), .clkout(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n224), .c(new_n223), .o1(new_n226));
  norp02aa1n02x5               g131(.a(new_n226), .b(new_n222), .o1(\s[22] ));
  nano23aa1n02x4               g132(.a(new_n216), .b(new_n219), .c(new_n220), .d(new_n217), .out0(new_n228));
  oaoi03aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .c(new_n223), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n212), .c(new_n228), .o1(new_n230));
  nona23aa1n02x4               g135(.a(new_n220), .b(new_n217), .c(new_n216), .d(new_n219), .out0(new_n231));
  nona32aa1n02x4               g136(.a(new_n193), .b(new_n231), .c(new_n202), .d(new_n209), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n184), .d(new_n192), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  norp02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n235), .b(new_n240), .c(new_n233), .d(new_n237), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n235), .clkout(new_n242));
  nanp02aa1n02x5               g147(.a(new_n233), .b(new_n237), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n240), .clkout(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n242), .o1(new_n245));
  norp02aa1n02x5               g150(.a(new_n245), .b(new_n241), .o1(\s[24] ));
  nano23aa1n02x4               g151(.a(new_n235), .b(new_n238), .c(new_n239), .d(new_n236), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n228), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  nona23aa1n02x4               g154(.a(new_n249), .b(new_n193), .c(new_n209), .d(new_n202), .out0(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .c(new_n242), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n247), .c(new_n229), .o1(new_n252));
  aobi12aa1n02x5               g157(.a(new_n252), .b(new_n249), .c(new_n212), .out0(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n250), .c(new_n184), .d(new_n192), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n256), .clkout(new_n260));
  nanp02aa1n02x5               g165(.a(new_n254), .b(new_n257), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n258), .clkout(new_n262));
  aoi012aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n263), .b(new_n259), .o1(\s[26] ));
  and002aa1n02x5               g169(.a(new_n258), .b(new_n257), .o(new_n265));
  nano32aa1n02x4               g170(.a(new_n210), .b(new_n265), .c(new_n228), .d(new_n247), .out0(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n266), .clkout(new_n267));
  nanp02aa1n02x5               g172(.a(new_n189), .b(new_n188), .o1(new_n268));
  oaoi03aa1n02x5               g173(.a(\a[18] ), .b(\b[17] ), .c(new_n268), .o1(new_n269));
  nanb03aa1n02x5               g174(.a(new_n202), .b(new_n269), .c(new_n201), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n252), .b(new_n248), .c(new_n270), .d(new_n211), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n272));
  aobi12aa1n02x5               g177(.a(new_n272), .b(new_n271), .c(new_n265), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n267), .c(new_n192), .d(new_n184), .o1(new_n274));
  xorb03aa1n02x5               g179(.a(new_n274), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  xorc02aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .out0(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n278), .clkout(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n265), .clkout(new_n280));
  oai012aa1n02x5               g185(.a(new_n272), .b(new_n253), .c(new_n280), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n281), .c(new_n185), .d(new_n266), .o1(new_n283));
  aoi012aa1n02x5               g188(.a(new_n279), .b(new_n283), .c(new_n277), .o1(new_n284));
  aoi112aa1n02x5               g189(.a(new_n276), .b(new_n278), .c(new_n274), .d(new_n282), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n284), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g191(.a(new_n279), .b(new_n277), .c(new_n282), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n281), .c(new_n185), .d(new_n266), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xorc02aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .out0(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n290), .clkout(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n288), .c(new_n289), .o1(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n289), .clkout(new_n293));
  aoi112aa1n02x5               g198(.a(new_n290), .b(new_n293), .c(new_n274), .d(new_n287), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n120), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g201(.a(new_n291), .b(new_n278), .c(new_n282), .d(new_n277), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n281), .c(new_n185), .d(new_n266), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n299));
  xorc02aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .out0(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .o1(new_n302));
  160nm_ficinv00aa1n08x5       g207(.clk(new_n299), .clkout(new_n303));
  aoi112aa1n02x5               g208(.a(new_n300), .b(new_n303), .c(new_n274), .d(new_n297), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  160nm_ficinv00aa1n08x5       g211(.clk(new_n306), .clkout(new_n307));
  and003aa1n02x5               g212(.a(new_n287), .b(new_n300), .c(new_n290), .o(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(new_n309), .clkout(new_n310));
  aoi112aa1n02x5               g215(.a(new_n307), .b(new_n310), .c(new_n274), .d(new_n308), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n308), .b(new_n281), .c(new_n185), .d(new_n266), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n306), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n122), .b(new_n116), .c(new_n117), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n122), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g223(.a(new_n110), .b(new_n124), .c(new_n109), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g225(.a(new_n125), .b(new_n111), .c(new_n124), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g227(.a(new_n104), .b(new_n321), .c(new_n105), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g229(.a(new_n175), .b(new_n128), .c(new_n101), .out0(\s[9] ));
endmodule


