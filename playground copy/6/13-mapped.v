// Benchmark "adder" written by ABC on Thu Jul 11 11:18:43 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n315, new_n317;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanb02aa1n02x5               g012(.a(new_n102), .b(new_n107), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nona22aa1n02x4               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[5] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[4] ), .clkout(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  160nm_fiao0012aa1n02p5x5     g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoib12aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .out0(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  norp02aa1n02x5               g033(.a(new_n128), .b(new_n125), .o1(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(\b[9] ), .clkout(new_n130));
  oao003aa1n02x5               g035(.a(new_n97), .b(new_n130), .c(new_n98), .carry(new_n131));
  aoi012aa1n02x5               g036(.a(new_n131), .b(new_n124), .c(new_n129), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g040(.a(\b[12] ), .b(\a[13] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[12] ), .b(\a[13] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  norp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1n02x4               g047(.a(new_n139), .b(new_n141), .c(new_n142), .d(new_n140), .out0(new_n143));
  aoi012aa1n02x5               g048(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n144));
  aobi12aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n131), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n109), .b(new_n146), .c(new_n102), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n148));
  norp03aa1n02x5               g053(.a(new_n148), .b(new_n115), .c(new_n116), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n121), .clkout(new_n150));
  oabi12aa1n02x5               g055(.a(new_n122), .b(new_n150), .c(new_n148), .out0(new_n151));
  nona23aa1n02x4               g056(.a(new_n142), .b(new_n140), .c(new_n139), .d(new_n141), .out0(new_n152));
  norp03aa1n02x5               g057(.a(new_n152), .b(new_n128), .c(new_n125), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n147), .d(new_n149), .o1(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n138), .b(new_n154), .c(new_n145), .out0(\s[13] ));
  oaoi03aa1n02x5               g060(.a(new_n97), .b(new_n130), .c(new_n98), .o1(new_n156));
  oai012aa1n02x5               g061(.a(new_n144), .b(new_n152), .c(new_n156), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n138), .b(new_n157), .c(new_n124), .d(new_n153), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n136), .out0(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1n02x4               g067(.a(new_n136), .b(new_n161), .c(new_n162), .d(new_n137), .out0(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  oai012aa1n02x5               g069(.a(new_n162), .b(new_n161), .c(new_n136), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n154), .d(new_n145), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nanp03aa1n02x5               g078(.a(new_n163), .b(new_n169), .c(new_n170), .o1(new_n174));
  nano22aa1n02x4               g079(.a(new_n174), .b(new_n129), .c(new_n143), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n151), .c(new_n147), .d(new_n149), .o1(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .out0(new_n177));
  xnrc02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n168), .clkout(new_n179));
  oao003aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n179), .carry(new_n180));
  oai013aa1n02x4               g085(.a(new_n180), .b(new_n178), .c(new_n177), .d(new_n165), .o1(new_n181));
  aoib12aa1n02x5               g086(.a(new_n181), .b(new_n157), .c(new_n174), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n176), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  oai022aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n02x5               g096(.a(new_n191), .b(new_n185), .c(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
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
  160nm_ficinv00aa1n08x5       g109(.clk(new_n194), .clkout(new_n205));
  aobi12aa1n02x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  nona23aa1n02x4               g114(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n210));
  aoi012aa1n02x5               g115(.a(new_n201), .b(new_n194), .c(new_n202), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n210), .c(new_n192), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n209), .c(new_n176), .d(new_n182), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[21] ), .clkout(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n190), .c(new_n208), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\b[21] ), .clkout(new_n226));
  oao003aa1n02x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .carry(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n212), .c(new_n224), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n225), .c(new_n176), .d(new_n182), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoi112aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(\s[24] ));
  oabi12aa1n02x5               g141(.a(new_n181), .b(new_n145), .c(new_n174), .out0(new_n237));
  and002aa1n02x5               g142(.a(new_n233), .b(new_n232), .o(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n224), .c(new_n190), .d(new_n208), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n237), .c(new_n124), .d(new_n175), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n211), .clkout(new_n242));
  aoai13aa1n02x5               g147(.a(new_n224), .b(new_n242), .c(new_n208), .d(new_n193), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n227), .clkout(new_n244));
  aoi112aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n245));
  oab012aa1n02x4               g150(.a(new_n245), .b(\a[24] ), .c(\b[23] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n239), .c(new_n243), .d(new_n244), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  xnrc02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n241), .c(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n250), .b(new_n247), .c(new_n183), .d(new_n240), .o1(new_n254));
  xnrc02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  nanp03aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n255), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n253), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[26] ));
  norp02aa1n02x5               g163(.a(new_n255), .b(new_n249), .o1(new_n259));
  nano22aa1n02x4               g164(.a(new_n225), .b(new_n238), .c(new_n259), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n237), .c(new_n124), .d(new_n175), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(new_n247), .b(new_n259), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aoi013aa1n02x4               g170(.a(new_n265), .b(new_n261), .c(new_n262), .d(new_n263), .o1(new_n266));
  aobi12aa1n02x5               g171(.a(new_n260), .b(new_n176), .c(new_n182), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n238), .b(new_n227), .c(new_n212), .d(new_n224), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n259), .clkout(new_n269));
  aoai13aa1n02x5               g174(.a(new_n263), .b(new_n269), .c(new_n268), .d(new_n246), .o1(new_n270));
  norp03aa1n02x5               g175(.a(new_n270), .b(new_n267), .c(new_n264), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n266), .b(new_n271), .o1(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n266), .b(new_n274), .c(new_n275), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n264), .b(new_n270), .c(new_n267), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n275), .b(new_n277), .c(new_n274), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n264), .b(new_n275), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n280), .b(new_n270), .c(new_n267), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n281), .c(new_n282), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n280), .clkout(new_n285));
  aoi013aa1n02x4               g190(.a(new_n285), .b(new_n261), .c(new_n262), .d(new_n263), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n286), .b(new_n282), .c(new_n283), .out0(new_n287));
  norp02aa1n02x5               g192(.a(new_n284), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n264), .b(new_n283), .c(new_n275), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n270), .c(new_n267), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n290), .clkout(new_n295));
  aoi013aa1n02x4               g200(.a(new_n295), .b(new_n261), .c(new_n262), .d(new_n263), .o1(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n292), .c(new_n293), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n294), .b(new_n297), .o1(\s[30] ));
  norb02aa1n02x5               g203(.a(new_n290), .b(new_n293), .out0(new_n299));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n299), .clkout(new_n300));
  aoi013aa1n02x4               g205(.a(new_n300), .b(new_n261), .c(new_n262), .d(new_n263), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n299), .b(new_n270), .c(new_n267), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n147), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g216(.a(new_n118), .b(new_n119), .c(new_n147), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g218(.a(new_n115), .b(new_n116), .o(new_n314));
  aoai13aa1n02x5               g219(.a(new_n150), .b(new_n314), .c(new_n108), .d(new_n109), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g221(.a(new_n112), .b(new_n315), .c(new_n113), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


