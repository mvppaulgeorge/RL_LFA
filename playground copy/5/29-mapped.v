// Benchmark "adder" written by ABC on Thu Jul 11 11:16:30 2024

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
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n314, new_n316, new_n318;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\b[3] ), .clkout(new_n111));
  aboi22aa1n03x5               g016(.a(\a[4] ), .b(new_n111), .c(new_n102), .d(new_n103), .out0(new_n112));
  oai012aa1n02x5               g017(.a(new_n112), .b(new_n110), .c(new_n106), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  nona23aa1n02x4               g026(.a(new_n113), .b(new_n121), .c(new_n118), .d(new_n101), .out0(new_n122));
  norp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n126));
  oabi12aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n125), .out0(new_n127));
  xnrc02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  nona22aa1n02x4               g033(.a(new_n122), .b(new_n127), .c(new_n128), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n98), .b(new_n129), .c(new_n100), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n98), .c(new_n129), .d(new_n100), .o1(new_n135));
  nano22aa1n02x4               g040(.a(new_n131), .b(new_n134), .c(new_n132), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n98), .c(new_n129), .d(new_n100), .o1(new_n137));
  aobi12aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(new_n133), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n131), .clkout(new_n139));
  xorc02aa1n02x5               g044(.a(\a[12] ), .b(\b[11] ), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n139), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n127), .clkout(new_n142));
  oai022aa1n02x5               g047(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n136), .b(new_n140), .c(new_n143), .d(new_n99), .out0(new_n144));
  nanp03aa1n02x5               g049(.a(new_n136), .b(new_n140), .c(new_n143), .o1(new_n145));
  oao003aa1n02x5               g050(.a(\a[12] ), .b(\b[11] ), .c(new_n139), .carry(new_n146));
  nanp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n144), .c(new_n122), .d(new_n142), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[14] ), .clkout(new_n151));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  aoi012aa1n02x5               g059(.a(new_n152), .b(new_n149), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnrc02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  norp02aa1n02x5               g065(.a(new_n160), .b(new_n153), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(\b[13] ), .clkout(new_n162));
  oaoi03aa1n02x5               g067(.a(new_n151), .b(new_n162), .c(new_n152), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoai13aa1n02x5               g069(.a(new_n159), .b(new_n164), .c(new_n149), .d(new_n161), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n159), .b(new_n164), .c(new_n149), .d(new_n161), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n157), .clkout(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n165), .c(new_n168), .out0(\s[16] ));
  oaoi13aa1n02x5               g077(.a(new_n101), .b(new_n112), .c(new_n110), .d(new_n106), .o1(new_n173));
  norp03aa1n02x5               g078(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n174));
  nano23aa1n02x4               g079(.a(new_n157), .b(new_n169), .c(new_n170), .d(new_n158), .out0(new_n175));
  nano22aa1n02x4               g080(.a(new_n144), .b(new_n161), .c(new_n175), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n127), .c(new_n173), .d(new_n174), .o1(new_n177));
  norb03aa1n02x5               g082(.a(new_n175), .b(new_n153), .c(new_n160), .out0(new_n178));
  aoi012aa1n02x5               g083(.a(new_n169), .b(new_n157), .c(new_n170), .o1(new_n179));
  oaib12aa1n02x5               g084(.a(new_n179), .b(new_n163), .c(new_n175), .out0(new_n180));
  aoi012aa1n02x5               g085(.a(new_n180), .b(new_n178), .c(new_n147), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n177), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nona22aa1n02x4               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(new_n191));
  oaib12aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n184), .out0(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x4               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  orn002aa1n02x5               g108(.a(\a[19] ), .b(\b[18] ), .o(new_n204));
  aobi12aa1n02x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n189), .b(new_n207), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  aoi013aa1n02x4               g114(.a(new_n209), .b(new_n190), .c(new_n185), .d(new_n186), .o1(new_n210));
  nona23aa1n02x4               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n211), .c(new_n210), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n208), .c(new_n177), .d(new_n181), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n189), .c(new_n207), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n226), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n177), .d(new_n181), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g143(.clk(\a[23] ), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\a[24] ), .clkout(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[22] ), .c(new_n240), .d(\b[23] ), .out0(new_n241));
  nano22aa1n02x4               g146(.a(new_n208), .b(new_n226), .c(new_n241), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n226), .b(new_n212), .c(new_n207), .d(new_n192), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n229), .clkout(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n241), .clkout(new_n245));
  oai022aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n246));
  oaib12aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(\b[23] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n244), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n242), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n242), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n02x4               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n253), .clkout(new_n256));
  aobi12aa1n02x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g163(.clk(\a[25] ), .clkout(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(\a[26] ), .clkout(new_n260));
  xroi22aa1d04x5               g165(.a(new_n259), .b(\b[24] ), .c(new_n260), .d(\b[25] ), .out0(new_n261));
  nano32aa1n02x4               g166(.a(new_n208), .b(new_n261), .c(new_n226), .d(new_n241), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n182), .b(new_n262), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n248), .c(new_n261), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n263), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n266), .clkout(new_n270));
  aoi012aa1n02x5               g175(.a(new_n270), .b(new_n265), .c(new_n263), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n269), .c(new_n272), .out0(new_n273));
  aobi12aa1n02x5               g178(.a(new_n262), .b(new_n177), .c(new_n181), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n241), .b(new_n229), .c(new_n214), .d(new_n226), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n261), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n264), .b(new_n276), .c(new_n275), .d(new_n247), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n266), .b(new_n277), .c(new_n274), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n272), .b(new_n278), .c(new_n269), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n273), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n266), .b(new_n272), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n274), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n281), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n265), .c(new_n263), .o1(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n283), .c(new_n284), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n285), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n266), .b(new_n284), .c(new_n272), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n277), .c(new_n274), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n291), .clkout(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n265), .c(new_n263), .o1(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n295), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n291), .b(new_n294), .out0(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n265), .c(new_n263), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n300), .b(new_n277), .c(new_n274), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  xnbna2aa1n03x5               g213(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n173), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g217(.a(\a[5] ), .b(\b[4] ), .o(new_n313));
  nona22aa1n02x4               g218(.a(new_n113), .b(new_n120), .c(new_n101), .out0(new_n314));
  xobna2aa1n03x5               g219(.a(new_n119), .b(new_n314), .c(new_n313), .out0(\s[6] ));
  aobi12aa1n02x5               g220(.a(new_n125), .b(new_n173), .c(new_n121), .out0(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g222(.a(\a[7] ), .b(\b[6] ), .c(new_n316), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g224(.a(new_n128), .b(new_n122), .c(new_n142), .out0(\s[9] ));
endmodule


