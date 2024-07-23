// Benchmark "adder" written by ABC on Thu Jul 11 12:58:02 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[2] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  oai012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\b[3] ), .clkout(new_n112));
  aboi22aa1n03x5               g017(.a(\a[4] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oaoi13aa1n02x5               g018(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  norp03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n126));
  oabi12aa1n02x5               g031(.a(new_n126), .b(new_n119), .c(new_n125), .out0(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n99), .clkout(new_n131));
  aoi012aa1n02x5               g036(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n131), .c(new_n129), .d(new_n101), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi112aa1n02x5               g044(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  oai012aa1n02x5               g047(.a(new_n113), .b(new_n111), .c(new_n107), .o1(new_n143));
  norp02aa1n02x5               g048(.a(new_n121), .b(new_n120), .o1(new_n144));
  nona23aa1n02x4               g049(.a(new_n143), .b(new_n144), .c(new_n119), .d(new_n102), .out0(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n127), .clkout(new_n146));
  nano23aa1n02x4               g051(.a(new_n135), .b(new_n137), .c(new_n138), .d(new_n136), .out0(new_n147));
  nanp03aa1n02x5               g052(.a(new_n147), .b(new_n99), .c(new_n128), .o1(new_n148));
  nona23aa1n02x4               g053(.a(new_n138), .b(new_n136), .c(new_n135), .d(new_n137), .out0(new_n149));
  aoi012aa1n02x5               g054(.a(new_n137), .b(new_n135), .c(new_n138), .o1(new_n150));
  oai012aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n132), .o1(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n148), .c(new_n145), .d(new_n146), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g059(.clk(\a[14] ), .clkout(new_n155));
  norp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  aoi012aa1n02x5               g063(.a(new_n156), .b(new_n153), .c(new_n158), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(new_n155), .out0(\s[14] ));
  norp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnrc02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  norp02aa1n02x5               g069(.a(new_n164), .b(new_n157), .o1(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(\b[13] ), .clkout(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n155), .b(new_n166), .c(new_n156), .o1(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  aoai13aa1n02x5               g073(.a(new_n163), .b(new_n168), .c(new_n153), .d(new_n165), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n163), .b(new_n168), .c(new_n153), .d(new_n165), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n161), .clkout(new_n172));
  norp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n169), .c(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n161), .b(new_n173), .c(new_n174), .d(new_n162), .out0(new_n177));
  nano22aa1n02x4               g082(.a(new_n148), .b(new_n165), .c(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n173), .b(new_n161), .c(new_n174), .o1(new_n180));
  oaib12aa1n02x5               g085(.a(new_n180), .b(new_n167), .c(new_n177), .out0(new_n181));
  aoi013aa1n02x4               g086(.a(new_n181), .b(new_n151), .c(new_n165), .d(new_n177), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nona22aa1n02x4               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n185), .out0(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x4               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  orn002aa1n02x5               g109(.a(\a[19] ), .b(\b[18] ), .o(new_n205));
  aobi12aa1n02x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  aoi013aa1n02x4               g115(.a(new_n210), .b(new_n191), .c(new_n186), .d(new_n187), .o1(new_n211));
  nona23aa1n02x4               g116(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  oai012aa1n02x5               g119(.a(new_n214), .b(new_n212), .c(new_n211), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n209), .c(new_n179), .d(new_n182), .o1(new_n217));
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
  nanp03aa1n02x5               g132(.a(new_n227), .b(new_n190), .c(new_n208), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\b[21] ), .clkout(new_n229));
  oao003aa1n02x5               g134(.a(new_n226), .b(new_n229), .c(new_n219), .carry(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n215), .c(new_n227), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n228), .c(new_n179), .d(new_n182), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g144(.clk(\a[23] ), .clkout(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(\a[24] ), .clkout(new_n241));
  xroi22aa1d04x5               g146(.a(new_n240), .b(\b[22] ), .c(new_n241), .d(\b[23] ), .out0(new_n242));
  nano22aa1n02x4               g147(.a(new_n209), .b(new_n227), .c(new_n242), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n227), .b(new_n213), .c(new_n208), .d(new_n193), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n230), .clkout(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n242), .clkout(new_n246));
  oai022aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n247));
  oaib12aa1n02x5               g152(.a(new_n247), .b(new_n241), .c(\b[23] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n245), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n249), .c(new_n183), .d(new_n243), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n183), .d(new_n243), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  nona22aa1n02x4               g160(.a(new_n251), .b(new_n255), .c(new_n254), .out0(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n254), .clkout(new_n257));
  aobi12aa1n02x5               g162(.a(new_n255), .b(new_n251), .c(new_n257), .out0(new_n258));
  norb02aa1n02x5               g163(.a(new_n256), .b(new_n258), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g164(.clk(\a[25] ), .clkout(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(\a[26] ), .clkout(new_n261));
  xroi22aa1d04x5               g166(.a(new_n260), .b(\b[24] ), .c(new_n261), .d(\b[25] ), .out0(new_n262));
  nano32aa1n02x4               g167(.a(new_n209), .b(new_n262), .c(new_n227), .d(new_n242), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n183), .b(new_n263), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n265));
  aobi12aa1n02x5               g170(.a(new_n265), .b(new_n249), .c(new_n262), .out0(new_n266));
  xorc02aa1n02x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n264), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n269), .clkout(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n267), .clkout(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n266), .c(new_n264), .o1(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n02x4               g178(.a(new_n272), .b(new_n270), .c(new_n273), .out0(new_n274));
  aobi12aa1n02x5               g179(.a(new_n263), .b(new_n179), .c(new_n182), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n242), .b(new_n230), .c(new_n215), .d(new_n227), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n262), .clkout(new_n277));
  aoai13aa1n02x5               g182(.a(new_n265), .b(new_n277), .c(new_n276), .d(new_n248), .o1(new_n278));
  oai012aa1n02x5               g183(.a(new_n267), .b(new_n278), .c(new_n275), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n273), .b(new_n279), .c(new_n270), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n267), .b(new_n273), .out0(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n266), .c(new_n264), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n270), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n284), .b(new_n285), .c(new_n286), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n282), .b(new_n278), .c(new_n275), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n286), .b(new_n288), .c(new_n285), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n267), .b(new_n286), .c(new_n273), .out0(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n292), .clkout(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n266), .c(new_n264), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n292), .b(new_n278), .c(new_n275), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n292), .b(new_n296), .out0(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n301), .clkout(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n266), .c(new_n264), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n301), .b(new_n278), .c(new_n275), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnbna2aa1n03x5               g214(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g218(.a(\a[5] ), .b(\b[4] ), .o(new_n314));
  nona22aa1n02x4               g219(.a(new_n143), .b(new_n121), .c(new_n102), .out0(new_n315));
  xobna2aa1n03x5               g220(.a(new_n120), .b(new_n315), .c(new_n314), .out0(\s[6] ));
  aobi12aa1n02x5               g221(.a(new_n125), .b(new_n114), .c(new_n144), .out0(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n128), .b(new_n145), .c(new_n146), .out0(\s[9] ));
endmodule


