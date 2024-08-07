// Benchmark "adder" written by ABC on Thu Jul 11 12:06:46 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n317;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[8] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[7] ), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  oaoi03aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  aoi012aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[6] ), .b(\a[7] ), .out0(new_n106));
  oai013aa1n02x4               g011(.a(new_n101), .b(new_n97), .c(new_n106), .d(new_n105), .o1(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\a[4] ), .clkout(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(\b[3] ), .clkout(new_n109));
  nanp02aa1n02x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  oaoi03aa1n02x5               g018(.a(new_n108), .b(new_n109), .c(new_n113), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  aoi012aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(new_n113), .b(new_n119), .out0(new_n120));
  oai013aa1n02x4               g025(.a(new_n114), .b(new_n118), .c(new_n120), .d(new_n112), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nona23aa1n02x4               g027(.a(new_n104), .b(new_n122), .c(new_n103), .d(new_n102), .out0(new_n123));
  norp03aa1n02x5               g028(.a(new_n123), .b(new_n106), .c(new_n97), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n107), .b(new_n121), .c(new_n124), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n129), .b(new_n128), .c(new_n130), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nano23aa1n02x4               g037(.a(new_n128), .b(new_n129), .c(new_n130), .d(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n134), .b(new_n131), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(\a[12] ), .clkout(new_n137));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n139), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(new_n137), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  160nm_fiao0012aa1n02p5x5     g048(.a(new_n142), .b(new_n138), .c(new_n143), .o(new_n144));
  nano23aa1n02x4               g049(.a(new_n138), .b(new_n142), .c(new_n143), .d(new_n139), .out0(new_n145));
  aoib12aa1n02x5               g050(.a(new_n144), .b(new_n145), .c(new_n131), .out0(new_n146));
  nona23aa1n02x4               g051(.a(new_n143), .b(new_n139), .c(new_n138), .d(new_n142), .out0(new_n147));
  norb02aa1n02x5               g052(.a(new_n133), .b(new_n147), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n146), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g056(.clk(\a[14] ), .clkout(new_n152));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n159));
  oabi12aa1n02x5               g064(.a(new_n144), .b(new_n147), .c(new_n131), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n162));
  aobi12aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n162), .out0(new_n163));
  oai012aa1n02x5               g068(.a(new_n163), .b(new_n149), .c(new_n159), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n166), .b(new_n170), .c(new_n164), .d(new_n167), .o1(new_n172));
  nanb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nanb02aa1n02x5               g078(.a(new_n166), .b(new_n167), .out0(new_n174));
  aoi012aa1n02x5               g079(.a(new_n168), .b(new_n166), .c(new_n169), .o1(new_n175));
  oai013aa1n02x4               g080(.a(new_n175), .b(new_n161), .c(new_n174), .d(new_n170), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n162), .o1(new_n178));
  aoib12aa1n02x5               g083(.a(new_n176), .b(new_n160), .c(new_n178), .out0(new_n179));
  nano22aa1n02x4               g084(.a(new_n178), .b(new_n133), .c(new_n145), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n181));
  xorc02aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n181), .c(new_n179), .out0(\s[17] ));
  nanp02aa1n02x5               g088(.a(new_n181), .b(new_n179), .o1(new_n184));
  norp02aa1n02x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  aoi012aa1n02x5               g090(.a(new_n185), .b(new_n184), .c(new_n182), .o1(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[17] ), .clkout(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n186), .b(new_n190), .c(new_n189), .out0(\s[18] ));
  aob012aa1n02x5               g096(.a(new_n189), .b(new_n185), .c(new_n190), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n182), .clkout(new_n194));
  nano22aa1n02x4               g099(.a(new_n194), .b(new_n189), .c(new_n190), .out0(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n195), .clkout(new_n196));
  aoai13aa1n02x5               g101(.a(new_n193), .b(new_n196), .c(new_n181), .d(new_n179), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n201), .b(new_n200), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n197), .b(new_n207), .o1(new_n208));
  nona22aa1n02x4               g113(.a(new_n208), .b(new_n205), .c(new_n200), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n209), .b(new_n206), .o1(\s[20] ));
  aoi012aa1n02x5               g115(.a(new_n202), .b(new_n200), .c(new_n203), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  nano23aa1n02x4               g117(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n213));
  aoi012aa1n02x5               g118(.a(new_n212), .b(new_n213), .c(new_n192), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n195), .b(new_n213), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n215), .c(new_n181), .d(new_n179), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  norp02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n218), .c(new_n216), .d(new_n220), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n216), .b(new_n220), .o1(new_n226));
  nona22aa1n02x4               g131(.a(new_n226), .b(new_n224), .c(new_n218), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n227), .b(new_n225), .o1(\s[22] ));
  nanp03aa1n02x5               g133(.a(new_n192), .b(new_n207), .c(new_n204), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n211), .o1(new_n230));
  160nm_fiao0012aa1n02p5x5     g135(.a(new_n221), .b(new_n218), .c(new_n222), .o(new_n231));
  nano23aa1n02x4               g136(.a(new_n218), .b(new_n221), .c(new_n222), .d(new_n219), .out0(new_n232));
  aoi012aa1n02x5               g137(.a(new_n231), .b(new_n230), .c(new_n232), .o1(new_n233));
  nanp03aa1n02x5               g138(.a(new_n195), .b(new_n213), .c(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n181), .d(new_n179), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  norp02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n237), .c(new_n235), .d(new_n239), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(new_n235), .b(new_n239), .o1(new_n244));
  nona22aa1n02x4               g149(.a(new_n244), .b(new_n242), .c(new_n237), .out0(new_n245));
  nanp02aa1n02x5               g150(.a(new_n245), .b(new_n243), .o1(\s[24] ));
  160nm_fiao0012aa1n02p5x5     g151(.a(new_n240), .b(new_n237), .c(new_n241), .o(new_n247));
  nano23aa1n02x4               g152(.a(new_n237), .b(new_n240), .c(new_n241), .d(new_n238), .out0(new_n248));
  aoi012aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(new_n231), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(new_n248), .b(new_n232), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n229), .d(new_n211), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n251), .clkout(new_n252));
  nanb03aa1n02x5               g157(.a(new_n250), .b(new_n195), .c(new_n213), .out0(new_n253));
  aoai13aa1n02x5               g158(.a(new_n252), .b(new_n253), .c(new_n181), .d(new_n179), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[25] ), .b(\a[26] ), .out0(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n254), .b(new_n257), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n258), .c(new_n256), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n261), .b(new_n259), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g167(.clk(\a[26] ), .clkout(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(\b[25] ), .clkout(new_n264));
  oaoi03aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n256), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  norb02aa1n02x5               g171(.a(new_n257), .b(new_n258), .out0(new_n267));
  aoi012aa1n02x5               g172(.a(new_n266), .b(new_n251), .c(new_n267), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n250), .clkout(new_n269));
  nanb03aa1n02x5               g174(.a(new_n215), .b(new_n267), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n268), .b(new_n270), .c(new_n181), .d(new_n179), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n251), .b(new_n267), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(new_n277), .b(new_n265), .o1(new_n278));
  nano32aa1n02x4               g183(.a(new_n215), .b(new_n267), .c(new_n232), .d(new_n248), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n274), .b(new_n278), .c(new_n184), .d(new_n279), .o1(new_n280));
  nona22aa1n02x4               g185(.a(new_n280), .b(new_n275), .c(new_n273), .out0(new_n281));
  nanp02aa1n02x5               g186(.a(new_n276), .b(new_n281), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g187(.clk(\a[28] ), .clkout(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(\b[27] ), .clkout(new_n284));
  oaoi03aa1n02x5               g189(.a(new_n283), .b(new_n284), .c(new_n273), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n285), .clkout(new_n286));
  norb02aa1n02x5               g191(.a(new_n274), .b(new_n275), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n278), .c(new_n184), .d(new_n279), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  nona22aa1n02x4               g194(.a(new_n288), .b(new_n289), .c(new_n286), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n289), .b(new_n286), .c(new_n271), .d(new_n287), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(new_n291), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oaoi03aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .o1(new_n294));
  norb03aa1n02x5               g199(.a(new_n274), .b(new_n289), .c(new_n275), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n294), .c(new_n271), .d(new_n295), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n295), .b(new_n278), .c(new_n184), .d(new_n279), .o1(new_n298));
  nona22aa1n02x4               g203(.a(new_n298), .b(new_n296), .c(new_n294), .out0(new_n299));
  nanp02aa1n02x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  nanb02aa1n02x5               g205(.a(new_n296), .b(new_n294), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(\b[29] ), .c(\a[30] ), .o1(new_n302));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n296), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n278), .c(new_n184), .d(new_n279), .o1(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nona22aa1n02x4               g210(.a(new_n304), .b(new_n305), .c(new_n302), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n305), .b(new_n302), .c(new_n271), .d(new_n303), .o1(new_n307));
  nanp02aa1n02x5               g212(.a(new_n307), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g217(.a(new_n103), .b(new_n121), .c(new_n122), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g219(.a(new_n105), .b(new_n123), .c(new_n121), .out0(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g221(.a(new_n100), .b(new_n315), .c(new_n106), .out0(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(new_n98), .out0(\s[8] ));
  xnrb03aa1n02x5               g223(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


