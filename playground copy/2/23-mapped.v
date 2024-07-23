// Benchmark "adder" written by ABC on Wed Jul 10 17:24:37 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n309, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  oai012aa1n02x5               g022(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[5] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[4] ), .clkout(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  oaib12aa1n02x5               g027(.a(new_n118), .b(new_n114), .c(new_n122), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aobi12aa1n02x5               g038(.a(new_n129), .b(new_n126), .c(new_n98), .out0(new_n134));
  oai012aa1n02x5               g039(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(new_n134), .b(new_n136), .o1(new_n137));
  xnrc02aa1n02x5               g042(.a(new_n137), .b(new_n133), .out0(\s[11] ));
  oai012aa1n02x5               g043(.a(new_n133), .b(new_n134), .c(new_n136), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n139), .b(new_n142), .c(new_n131), .out0(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n142), .clkout(new_n144));
  oaoi13aa1n02x5               g049(.a(new_n144), .b(new_n139), .c(\a[11] ), .d(\b[10] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n143), .b(new_n145), .out0(\s[12] ));
  nanp02aa1n02x5               g051(.a(new_n109), .b(new_n117), .o1(new_n147));
  oai022aa1n02x5               g052(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n148));
  aboi22aa1n03x5               g053(.a(new_n114), .b(new_n122), .c(new_n111), .d(new_n148), .out0(new_n149));
  nano23aa1n02x4               g054(.a(new_n131), .b(new_n140), .c(new_n141), .d(new_n132), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n150), .b(new_n125), .c(new_n129), .o1(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n140), .clkout(new_n152));
  nona23aa1n02x4               g057(.a(new_n141), .b(new_n132), .c(new_n131), .d(new_n140), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n131), .b(new_n141), .o1(new_n154));
  oai112aa1n02x5               g059(.a(new_n154), .b(new_n152), .c(new_n153), .d(new_n135), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n151), .c(new_n147), .d(new_n149), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  norp02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nano23aa1n02x4               g073(.a(new_n159), .b(new_n167), .c(new_n168), .d(new_n160), .out0(new_n169));
  oai012aa1n02x5               g074(.a(new_n168), .b(new_n167), .c(new_n159), .o1(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  aoai13aa1n02x5               g076(.a(new_n166), .b(new_n171), .c(new_n157), .d(new_n169), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n166), .b(new_n171), .c(new_n157), .d(new_n169), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n163), .clkout(new_n175));
  norp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .out0(\s[16] ));
  nano23aa1n02x4               g084(.a(new_n163), .b(new_n176), .c(new_n177), .d(new_n164), .out0(new_n180));
  nano22aa1n02x4               g085(.a(new_n151), .b(new_n169), .c(new_n180), .out0(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n182));
  and002aa1n02x5               g087(.a(new_n180), .b(new_n169), .o(new_n183));
  aoi112aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n184));
  obai22aa1n02x7               g089(.a(new_n180), .b(new_n170), .c(\a[16] ), .d(\b[15] ), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n184), .c(new_n155), .d(new_n183), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(new_n191), .b(new_n190), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  nona22aa1n02x4               g111(.a(new_n200), .b(new_n206), .c(new_n197), .out0(new_n207));
  orn002aa1n02x5               g112(.a(\a[19] ), .b(\b[18] ), .o(new_n208));
  aobi12aa1n02x5               g113(.a(new_n206), .b(new_n200), .c(new_n208), .out0(new_n209));
  norb02aa1n02x5               g114(.a(new_n207), .b(new_n209), .out0(\s[20] ));
  nano23aa1n02x4               g115(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n198), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n194), .b(new_n211), .o1(new_n212));
  oai022aa1n02x5               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  oaib12aa1n02x5               g118(.a(new_n213), .b(new_n189), .c(\b[17] ), .out0(new_n214));
  nona23aa1n02x4               g119(.a(new_n205), .b(new_n198), .c(new_n197), .d(new_n204), .out0(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[20] ), .b(\b[19] ), .c(new_n208), .o1(new_n216));
  oabi12aa1n02x5               g121(.a(new_n216), .b(new_n215), .c(new_n214), .out0(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n212), .c(new_n186), .d(new_n182), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g131(.clk(\a[21] ), .clkout(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\a[22] ), .clkout(new_n228));
  xroi22aa1d04x5               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nanp03aa1n02x5               g134(.a(new_n229), .b(new_n194), .c(new_n211), .o1(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oaoi03aa1n02x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .o1(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n217), .c(new_n229), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n230), .c(new_n186), .d(new_n182), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  and002aa1n02x5               g147(.a(new_n239), .b(new_n238), .o(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  nano32aa1n02x4               g149(.a(new_n244), .b(new_n229), .c(new_n194), .d(new_n211), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n229), .b(new_n216), .c(new_n211), .d(new_n196), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n247));
  oab012aa1n02x4               g152(.a(new_n247), .b(\a[24] ), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n244), .c(new_n246), .d(new_n232), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n249), .c(new_n187), .d(new_n245), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n187), .d(new_n245), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  nona22aa1n02x4               g160(.a(new_n251), .b(new_n255), .c(new_n254), .out0(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n254), .clkout(new_n257));
  aobi12aa1n02x5               g162(.a(new_n255), .b(new_n251), .c(new_n257), .out0(new_n258));
  norb02aa1n02x5               g163(.a(new_n256), .b(new_n258), .out0(\s[26] ));
  and002aa1n02x5               g164(.a(new_n255), .b(new_n250), .o(new_n260));
  nano22aa1n02x4               g165(.a(new_n230), .b(new_n243), .c(new_n260), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n187), .b(new_n261), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n263));
  aobi12aa1n02x5               g168(.a(new_n263), .b(new_n249), .c(new_n260), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n264), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  aobi12aa1n02x5               g173(.a(new_n265), .b(new_n262), .c(new_n264), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n261), .clkout(new_n272));
  aoi012aa1n02x5               g177(.a(new_n272), .b(new_n186), .c(new_n182), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n243), .b(new_n233), .c(new_n217), .d(new_n229), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n260), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n263), .b(new_n275), .c(new_n274), .d(new_n248), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n265), .b(new_n276), .c(new_n273), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n270), .b(new_n277), .c(new_n268), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n265), .b(new_n270), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n273), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n281), .c(new_n282), .o1(new_n284));
  aobi12aa1n02x5               g189(.a(new_n280), .b(new_n262), .c(new_n264), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n282), .c(new_n283), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n265), .b(new_n283), .c(new_n270), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n276), .c(new_n273), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  aobi12aa1n02x5               g198(.a(new_n289), .b(new_n262), .c(new_n264), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  norp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[30] ));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  norb02aa1n02x5               g202(.a(new_n289), .b(new_n292), .out0(new_n298));
  aobi12aa1n02x5               g203(.a(new_n298), .b(new_n262), .c(new_n264), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n300));
  nano22aa1n02x4               g205(.a(new_n299), .b(new_n297), .c(new_n300), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n298), .b(new_n276), .c(new_n273), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n297), .b(new_n302), .c(new_n300), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n303), .b(new_n301), .o1(\s[31] ));
  xnrb03aa1n02x5               g209(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g213(.a(new_n119), .b(new_n120), .c(new_n109), .o1(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g215(.a(\a[6] ), .b(\b[5] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g217(.a(new_n112), .b(new_n311), .c(new_n113), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g219(.a(new_n125), .b(new_n147), .c(new_n149), .out0(\s[9] ));
endmodule


