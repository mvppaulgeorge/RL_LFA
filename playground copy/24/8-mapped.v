// Benchmark "adder" written by ABC on Thu Jul 11 12:38:30 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  aoi012aa1n02x5               g012(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(new_n108), .clkout(new_n109));
  160nm_fiao0012aa1n02p5x5     g014(.a(new_n100), .b(new_n102), .c(new_n101), .o(new_n110));
  aoi012aa1n02x5               g015(.a(new_n110), .b(new_n104), .c(new_n109), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  aoi012aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(new_n115), .clkout(new_n116));
  norp02aa1n02x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[3] ), .b(\a[4] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[2] ), .b(\a[3] ), .o1(new_n120));
  nano23aa1n02x4               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n117), .b(new_n119), .c(new_n118), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nano23aa1n02x4               g029(.a(new_n105), .b(new_n107), .c(new_n124), .d(new_n106), .out0(new_n125));
  nanp02aa1n02x5               g030(.a(new_n125), .b(new_n104), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n111), .b(new_n126), .c(new_n122), .d(new_n123), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n127), .c(new_n99), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n128), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n130), .b(new_n134), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n137), .c(new_n132), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n138), .o1(\s[12] ));
  nano23aa1n02x4               g046(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n142));
  orn002aa1n02x5               g047(.a(\a[9] ), .b(\b[8] ), .o(new_n143));
  oaoi03aa1n02x5               g048(.a(\a[10] ), .b(\b[9] ), .c(new_n143), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n142), .b(new_n144), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n135), .b(new_n132), .c(new_n136), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  nona23aa1n02x4               g053(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n110), .b(new_n149), .c(new_n108), .out0(new_n150));
  nona23aa1n02x4               g055(.a(new_n120), .b(new_n118), .c(new_n117), .d(new_n119), .out0(new_n151));
  oai012aa1n02x5               g056(.a(new_n123), .b(new_n151), .c(new_n115), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n125), .b(new_n149), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[10] ), .b(\b[9] ), .out0(new_n154));
  norb02aa1n02x5               g059(.a(new_n99), .b(new_n98), .out0(new_n155));
  nano32aa1n02x4               g060(.a(new_n137), .b(new_n154), .c(new_n155), .d(new_n134), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n150), .c(new_n153), .d(new_n152), .o1(new_n157));
  xorc02aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n157), .c(new_n148), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  aoai13aa1n02x5               g065(.a(new_n158), .b(new_n147), .c(new_n127), .d(new_n156), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n160), .out0(\s[14] ));
  and002aa1n02x5               g068(.a(new_n162), .b(new_n158), .o(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  oaoi03aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n157), .d(new_n148), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n174), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  nano23aa1n02x4               g082(.a(new_n170), .b(new_n172), .c(new_n173), .d(new_n171), .out0(new_n178));
  160nm_fiao0012aa1n02p5x5     g083(.a(new_n172), .b(new_n170), .c(new_n173), .o(new_n179));
  aoi012aa1n02x5               g084(.a(new_n179), .b(new_n178), .c(new_n166), .o1(new_n180));
  nanp03aa1n02x5               g085(.a(new_n178), .b(new_n158), .c(new_n162), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n145), .d(new_n146), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  nano32aa1n02x4               g088(.a(new_n181), .b(new_n155), .c(new_n142), .d(new_n154), .out0(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n150), .c(new_n153), .d(new_n152), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n183), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[17] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\b[16] ), .clkout(new_n190));
  oaoi03aa1n02x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d04x5               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  norp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n196));
  norp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n183), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  norp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n205), .clkout(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n201), .c(new_n198), .d(new_n202), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n202), .b(new_n201), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n198), .b(new_n208), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n206), .c(new_n201), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n207), .o1(\s[20] ));
  nona23aa1n02x4               g116(.a(new_n204), .b(new_n202), .c(new_n201), .d(new_n203), .out0(new_n212));
  aoi012aa1n02x5               g117(.a(new_n203), .b(new_n201), .c(new_n204), .o1(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n212), .c(new_n197), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  nanb02aa1n02x5               g120(.a(new_n212), .b(new_n193), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n216), .c(new_n185), .d(new_n183), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(new_n217), .b(new_n220), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n223), .b(new_n221), .c(new_n219), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  oai112aa1n02x5               g130(.a(new_n208), .b(new_n205), .c(new_n196), .d(new_n195), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n221), .b(new_n220), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\a[22] ), .clkout(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\b[21] ), .clkout(new_n229));
  oaoi03aa1n02x5               g134(.a(new_n228), .b(new_n229), .c(new_n219), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n226), .d(new_n213), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  nona23aa1n02x4               g137(.a(new_n193), .b(new_n220), .c(new_n221), .d(new_n212), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n232), .b(new_n233), .c(new_n185), .d(new_n183), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n234), .b(new_n237), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n239), .c(new_n236), .out0(new_n242));
  nanp02aa1n02x5               g147(.a(new_n242), .b(new_n240), .o1(\s[24] ));
  norb02aa1n02x5               g148(.a(new_n220), .b(new_n221), .out0(new_n244));
  and002aa1n02x5               g149(.a(new_n238), .b(new_n237), .o(new_n245));
  nano22aa1n02x4               g150(.a(new_n216), .b(new_n245), .c(new_n244), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n182), .c(new_n127), .d(new_n184), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n230), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n245), .b(new_n248), .c(new_n214), .d(new_n244), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(\a[24] ), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(\b[23] ), .clkout(new_n251));
  oaoi03aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n236), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n249), .b(new_n252), .o1(new_n253));
  nanb02aa1n02x5               g158(.a(new_n253), .b(new_n247), .out0(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n260));
  aoai13aa1n02x5               g165(.a(new_n257), .b(new_n253), .c(new_n186), .d(new_n246), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n259), .c(new_n256), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n252), .clkout(new_n264));
  and002aa1n02x5               g169(.a(new_n258), .b(new_n257), .o(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n264), .c(new_n231), .d(new_n245), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n267));
  oab012aa1n02x4               g172(.a(new_n267), .b(\a[26] ), .c(\b[25] ), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n233), .b(new_n245), .c(new_n265), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n182), .c(new_n127), .d(new_n184), .o1(new_n270));
  nanp03aa1n02x5               g175(.a(new_n270), .b(new_n266), .c(new_n268), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n265), .clkout(new_n277));
  aoai13aa1n02x5               g182(.a(new_n268), .b(new_n277), .c(new_n249), .d(new_n252), .o1(new_n278));
  aobi12aa1n02x5               g183(.a(new_n269), .b(new_n185), .c(new_n183), .out0(new_n279));
  oai012aa1n02x5               g184(.a(new_n274), .b(new_n278), .c(new_n279), .o1(new_n280));
  nona22aa1n02x4               g185(.a(new_n280), .b(new_n275), .c(new_n273), .out0(new_n281));
  nanp02aa1n02x5               g186(.a(new_n276), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g187(.a(new_n274), .b(new_n275), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n278), .c(new_n279), .o1(new_n284));
  aob012aa1n02x5               g189(.a(new_n273), .b(\b[27] ), .c(\a[28] ), .out0(new_n285));
  oa0012aa1n02x5               g190(.a(new_n285), .b(\b[27] ), .c(\a[28] ), .o(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nona22aa1n02x4               g193(.a(new_n284), .b(new_n287), .c(new_n288), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n288), .b(new_n287), .c(new_n271), .d(new_n283), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n290), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n274), .b(new_n288), .c(new_n275), .out0(new_n293));
  oaoi03aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .o1(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n294), .c(new_n271), .d(new_n293), .o1(new_n296));
  oai012aa1n02x5               g201(.a(new_n293), .b(new_n278), .c(new_n279), .o1(new_n297));
  nona22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  nanp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nanb02aa1n02x5               g204(.a(new_n295), .b(new_n294), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(\b[29] ), .c(\a[30] ), .o1(new_n301));
  norb02aa1n02x5               g206(.a(new_n293), .b(new_n295), .out0(new_n302));
  oai012aa1n02x5               g207(.a(new_n302), .b(new_n278), .c(new_n279), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nona22aa1n02x4               g209(.a(new_n303), .b(new_n304), .c(new_n301), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n304), .b(new_n301), .c(new_n271), .d(new_n302), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n306), .b(new_n305), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n152), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g216(.a(new_n107), .b(new_n152), .c(new_n124), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g218(.a(new_n108), .b(new_n152), .c(new_n125), .out0(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n102), .b(new_n314), .c(new_n103), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


