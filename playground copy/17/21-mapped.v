// Benchmark "adder" written by ABC on Thu Jul 11 12:09:18 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n316;
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
  aoi012aa1n02x5               g026(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nanp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  xorc02aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n126), .b(new_n129), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n128), .out0(\s[11] ));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi113aa1n02x5               g042(.a(new_n137), .b(new_n131), .c(new_n130), .d(new_n133), .e(new_n128), .o1(new_n138));
  nanp03aa1n02x5               g043(.a(new_n130), .b(new_n128), .c(new_n133), .o1(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n137), .clkout(new_n140));
  oaoi13aa1n02x5               g045(.a(new_n140), .b(new_n139), .c(\a[11] ), .d(\b[10] ), .o1(new_n141));
  norp02aa1n02x5               g046(.a(new_n141), .b(new_n138), .o1(\s[12] ));
  nona23aa1n02x4               g047(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n109), .b(new_n143), .c(new_n102), .o1(new_n144));
  nona23aa1n02x4               g049(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n145));
  norp03aa1n02x5               g050(.a(new_n145), .b(new_n115), .c(new_n116), .o1(new_n146));
  oaib12aa1n02x5               g051(.a(new_n122), .b(new_n145), .c(new_n121), .out0(new_n147));
  nano23aa1n02x4               g052(.a(new_n131), .b(new_n135), .c(new_n136), .d(new_n132), .out0(new_n148));
  and003aa1n02x5               g053(.a(new_n148), .b(new_n129), .c(new_n125), .o(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n144), .d(new_n146), .o1(new_n150));
  orn002aa1n02x5               g055(.a(\a[9] ), .b(\b[8] ), .o(new_n151));
  oaoi03aa1n02x5               g056(.a(\a[10] ), .b(\b[9] ), .c(new_n151), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n148), .b(new_n152), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n135), .b(new_n131), .c(new_n136), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n153), .b(new_n154), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  xorc02aa1n02x5               g061(.a(\a[13] ), .b(\b[12] ), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n150), .c(new_n156), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  aoai13aa1n02x5               g064(.a(new_n157), .b(new_n155), .c(new_n124), .d(new_n149), .o1(new_n160));
  xorc02aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n159), .out0(\s[14] ));
  nanp02aa1n02x5               g067(.a(new_n161), .b(new_n157), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n150), .d(new_n156), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoi112aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n02x4               g080(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n176));
  nanp03aa1n02x5               g081(.a(new_n176), .b(new_n157), .c(new_n161), .o1(new_n177));
  nano32aa1n02x4               g082(.a(new_n177), .b(new_n148), .c(new_n129), .d(new_n125), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n147), .c(new_n144), .d(new_n146), .o1(new_n179));
  160nm_fiao0012aa1n02p5x5     g084(.a(new_n170), .b(new_n168), .c(new_n171), .o(new_n180));
  aoi012aa1n02x5               g085(.a(new_n180), .b(new_n176), .c(new_n164), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n177), .c(new_n153), .d(new_n154), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  nanp02aa1n02x5               g088(.a(new_n179), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  oai022aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(new_n186), .c(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n02x4               g109(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n195), .clkout(new_n206));
  aobi12aa1n02x5               g111(.a(new_n204), .b(new_n198), .c(new_n206), .out0(new_n207));
  norb02aa1n02x5               g112(.a(new_n205), .b(new_n207), .out0(\s[20] ));
  nano23aa1n02x4               g113(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n191), .b(new_n209), .o1(new_n210));
  nona23aa1n02x4               g115(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n211));
  aoi012aa1n02x5               g116(.a(new_n202), .b(new_n195), .c(new_n203), .o1(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n193), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n210), .c(new_n179), .d(new_n183), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n191), .c(new_n209), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oaoi03aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n213), .c(new_n225), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n226), .c(new_n179), .d(new_n183), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n225), .c(new_n191), .d(new_n209), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n182), .c(new_n124), .d(new_n178), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n212), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n225), .b(new_n243), .c(new_n209), .d(new_n194), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n245));
  oab012aa1n02x4               g150(.a(new_n245), .b(\a[24] ), .c(\b[23] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(new_n244), .d(new_n228), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  xnrc02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n242), .c(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n250), .b(new_n247), .c(new_n184), .d(new_n241), .o1(new_n254));
  xnrc02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  nanp03aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n255), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n253), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[26] ));
  norp02aa1n02x5               g163(.a(new_n255), .b(new_n249), .o1(new_n259));
  nano22aa1n02x4               g164(.a(new_n226), .b(new_n239), .c(new_n259), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n182), .c(new_n124), .d(new_n178), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(new_n247), .b(new_n259), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aoi013aa1n02x4               g170(.a(new_n265), .b(new_n261), .c(new_n262), .d(new_n263), .o1(new_n266));
  aobi12aa1n02x5               g171(.a(new_n260), .b(new_n179), .c(new_n183), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n239), .b(new_n229), .c(new_n213), .d(new_n225), .o1(new_n268));
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
  xorb03aa1n02x5               g215(.a(new_n144), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g216(.a(new_n118), .b(new_n119), .c(new_n144), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g218(.a(\a[6] ), .b(\b[5] ), .c(new_n312), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n112), .b(new_n314), .c(new_n113), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


