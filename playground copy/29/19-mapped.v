// Benchmark "adder" written by ABC on Thu Jul 11 13:02:20 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n308, new_n310, new_n312, new_n314;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[9] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[8] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  and002aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o(new_n103));
  nanp02aa1n02x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oab012aa1n02x4               g010(.a(new_n103), .b(new_n105), .c(new_n104), .out0(new_n106));
  xorc02aa1n02x5               g011(.a(\a[4] ), .b(\b[3] ), .out0(new_n107));
  norp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nanp03aa1n02x5               g015(.a(new_n106), .b(new_n107), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(new_n108), .clkout(new_n112));
  oao003aa1n02x5               g017(.a(\a[4] ), .b(\b[3] ), .c(new_n112), .carry(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nano23aa1n02x4               g027(.a(new_n121), .b(new_n120), .c(new_n122), .d(new_n119), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n118), .o1(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n120), .b(new_n121), .c(new_n119), .o(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n126));
  aoi012aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n125), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n124), .c(new_n111), .d(new_n113), .o1(new_n128));
  oaib12aa1n02x5               g033(.a(new_n128), .b(new_n101), .c(\a[9] ), .out0(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n102), .out0(\s[10] ));
  aoai13aa1n02x5               g035(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n99), .c(new_n129), .d(new_n102), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  nona23aa1n02x4               g046(.a(new_n137), .b(new_n135), .c(new_n134), .d(new_n136), .out0(new_n142));
  160nm_fiao0012aa1n02p5x5     g047(.a(new_n136), .b(new_n134), .c(new_n137), .o(new_n143));
  oabi12aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n131), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(\b[8] ), .b(\a[9] ), .out0(new_n146));
  nona22aa1n02x4               g051(.a(new_n145), .b(new_n146), .c(new_n99), .out0(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  xnrc02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n144), .c(new_n128), .d(new_n148), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(new_n144), .b(new_n150), .c(new_n128), .d(new_n148), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(\s[13] ));
  orn002aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .o(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n151), .c(new_n154), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(new_n155), .b(new_n149), .o1(new_n157));
  oao003aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .carry(new_n158));
  aobi12aa1n02x5               g063(.a(new_n158), .b(new_n144), .c(new_n157), .out0(new_n159));
  nona32aa1n02x4               g064(.a(new_n128), .b(new_n155), .c(new_n149), .d(new_n147), .out0(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n159), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  160nm_fiao0012aa1n02p5x5     g068(.a(new_n161), .b(new_n160), .c(new_n159), .o(new_n164));
  xnrc02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  oaib12aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n164), .out0(new_n166));
  nona22aa1n02x4               g071(.a(new_n164), .b(new_n165), .c(new_n163), .out0(new_n167));
  nanp02aa1n02x5               g072(.a(new_n166), .b(new_n167), .o1(\s[16] ));
  160nm_ficinv00aa1n08x5       g073(.clk(\a[17] ), .clkout(new_n169));
  norp02aa1n02x5               g074(.a(new_n165), .b(new_n161), .o1(new_n170));
  nano22aa1n02x4               g075(.a(new_n147), .b(new_n157), .c(new_n170), .out0(new_n171));
  oaoi03aa1n02x5               g076(.a(\a[10] ), .b(\b[9] ), .c(new_n102), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n157), .b(new_n143), .c(new_n145), .d(new_n172), .o1(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n170), .clkout(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n158), .o1(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(\a[16] ), .clkout(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(\b[15] ), .clkout(new_n177));
  oaoi03aa1n02x5               g082(.a(new_n176), .b(new_n177), .c(new_n163), .o1(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  aoi112aa1n02x5               g084(.a(new_n175), .b(new_n179), .c(new_n128), .d(new_n171), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(new_n169), .out0(\s[17] ));
  oaoi03aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .c(new_n180), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  xroi22aa1d04x5               g089(.a(new_n169), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  aoi112aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n187));
  aoib12aa1n02x5               g092(.a(new_n187), .b(new_n184), .c(\b[17] ), .out0(new_n188));
  oai012aa1n02x5               g093(.a(new_n188), .b(new_n180), .c(new_n186), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n192), .c(new_n189), .d(new_n194), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n128), .b(new_n171), .o1(new_n199));
  oai112aa1n02x5               g104(.a(new_n199), .b(new_n178), .c(new_n174), .d(new_n159), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  aob012aa1n02x5               g106(.a(new_n201), .b(\b[17] ), .c(\a[18] ), .out0(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(new_n184), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n194), .b(new_n203), .c(new_n200), .d(new_n185), .o1(new_n204));
  nona22aa1n02x4               g109(.a(new_n204), .b(new_n197), .c(new_n192), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n198), .b(new_n205), .o1(\s[20] ));
  nona23aa1n02x4               g111(.a(new_n196), .b(new_n193), .c(new_n192), .d(new_n195), .out0(new_n207));
  160nm_fiao0012aa1n02p5x5     g112(.a(new_n195), .b(new_n192), .c(new_n196), .o(new_n208));
  oabi12aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n188), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  norb02aa1n02x5               g115(.a(new_n185), .b(new_n207), .out0(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  oai012aa1n02x5               g117(.a(new_n210), .b(new_n180), .c(new_n212), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xnrc02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .out0(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  xnrc02aa1n02x5               g122(.a(\b[21] ), .b(\a[22] ), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n215), .c(new_n213), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n217), .b(new_n209), .c(new_n200), .d(new_n211), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n218), .c(new_n215), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n219), .b(new_n221), .o1(\s[22] ));
  norp02aa1n02x5               g127(.a(new_n218), .b(new_n216), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\b[21] ), .clkout(new_n225));
  oaoi03aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n215), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n209), .c(new_n223), .o1(new_n228));
  nano23aa1n02x4               g133(.a(new_n192), .b(new_n195), .c(new_n196), .d(new_n193), .out0(new_n229));
  nano22aa1n02x4               g134(.a(new_n186), .b(new_n223), .c(new_n229), .out0(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  oai012aa1n02x5               g136(.a(new_n228), .b(new_n180), .c(new_n231), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xnrc02aa1n02x5               g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n228), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n235), .b(new_n238), .c(new_n200), .d(new_n230), .o1(new_n239));
  nona22aa1n02x4               g144(.a(new_n239), .b(new_n236), .c(new_n234), .out0(new_n240));
  nanp02aa1n02x5               g145(.a(new_n237), .b(new_n240), .o1(\s[24] ));
  norb02aa1n02x5               g146(.a(new_n235), .b(new_n236), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n185), .c(new_n223), .d(new_n229), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n223), .b(new_n208), .c(new_n229), .d(new_n203), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n247));
  oab012aa1n02x4               g152(.a(new_n247), .b(\a[24] ), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n243), .c(new_n246), .d(new_n226), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  oai012aa1n02x5               g155(.a(new_n250), .b(new_n180), .c(new_n245), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xnrc02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n254), .b(new_n249), .c(new_n200), .d(new_n244), .o1(new_n257));
  nona22aa1n02x4               g162(.a(new_n257), .b(new_n255), .c(new_n253), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(new_n256), .b(new_n258), .o1(\s[26] ));
  norb02aa1n02x5               g164(.a(new_n254), .b(new_n255), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  nona22aa1n02x4               g166(.a(new_n230), .b(new_n261), .c(new_n243), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(\a[26] ), .clkout(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(\b[25] ), .clkout(new_n264));
  oaoi03aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n253), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n249), .c(new_n260), .o1(new_n267));
  oai012aa1n02x5               g172(.a(new_n267), .b(new_n180), .c(new_n262), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n262), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n242), .b(new_n227), .c(new_n209), .d(new_n223), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n265), .b(new_n261), .c(new_n275), .d(new_n248), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n271), .b(new_n276), .c(new_n200), .d(new_n274), .o1(new_n277));
  nona22aa1n02x4               g182(.a(new_n277), .b(new_n272), .c(new_n270), .out0(new_n278));
  nanp02aa1n02x5               g183(.a(new_n273), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n271), .b(new_n272), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n200), .d(new_n274), .o1(new_n281));
  aob012aa1n02x5               g186(.a(new_n270), .b(\b[27] ), .c(\a[28] ), .out0(new_n282));
  oa0012aa1n02x5               g187(.a(new_n282), .b(\b[27] ), .c(\a[28] ), .o(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nona22aa1n02x4               g190(.a(new_n281), .b(new_n284), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n285), .b(new_n284), .c(new_n268), .d(new_n280), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n287), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n271), .b(new_n285), .c(new_n272), .out0(new_n290));
  oaoi03aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n291), .c(new_n268), .d(new_n290), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n290), .b(new_n276), .c(new_n200), .d(new_n274), .o1(new_n294));
  nona22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  nanp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[30] ));
  norb02aa1n02x5               g201(.a(new_n290), .b(new_n292), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n276), .c(new_n200), .d(new_n274), .o1(new_n298));
  nanb02aa1n02x5               g203(.a(new_n292), .b(new_n291), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(\b[29] ), .c(\a[30] ), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nona22aa1n02x4               g206(.a(new_n298), .b(new_n300), .c(new_n301), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n301), .b(new_n300), .c(new_n268), .d(new_n297), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(new_n303), .b(new_n302), .o1(\s[31] ));
  xobna2aa1n03x5               g209(.a(new_n106), .b(new_n109), .c(new_n112), .out0(\s[3] ));
  oai012aa1n02x5               g210(.a(new_n109), .b(new_n106), .c(new_n108), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g212(.a(new_n111), .b(new_n113), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g214(.a(new_n121), .b(new_n308), .c(new_n122), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g216(.a(new_n125), .b(new_n308), .c(new_n123), .o(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g218(.a(new_n116), .b(new_n312), .c(new_n117), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g220(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


