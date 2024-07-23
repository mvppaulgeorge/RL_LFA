// Benchmark "adder" written by ABC on Thu Jul 11 11:43:34 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n320;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(\a[5] ), .clkout(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[4] ), .clkout(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n112), .c(new_n119), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n107), .c(new_n115), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nano23aa1n02x4               g033(.a(new_n125), .b(new_n127), .c(new_n128), .d(new_n126), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n130));
  oai012aa1n02x5               g035(.a(new_n126), .b(new_n127), .c(new_n125), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n130), .c(new_n131), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n129), .clkout(new_n136));
  oai012aa1n02x5               g041(.a(new_n131), .b(new_n122), .c(new_n136), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n132), .b(new_n137), .c(new_n133), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n141), .b(new_n133), .c(new_n132), .d(new_n140), .out0(new_n142));
  160nm_fiao0012aa1n02p5x5     g047(.a(new_n140), .b(new_n132), .c(new_n141), .o(new_n143));
  oabi12aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n131), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n132), .b(new_n140), .c(new_n141), .d(new_n133), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n129), .o1(new_n146));
  oabi12aa1n02x5               g051(.a(new_n144), .b(new_n122), .c(new_n146), .out0(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g053(.clk(\a[14] ), .clkout(new_n149));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  aoib12aa1n02x5               g056(.a(new_n150), .b(new_n147), .c(new_n151), .out0(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(new_n149), .out0(\s[14] ));
  norp02aa1n02x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(\b[13] ), .clkout(new_n158));
  oaoi03aa1n02x5               g063(.a(new_n149), .b(new_n158), .c(new_n150), .o1(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  norp02aa1n02x5               g066(.a(new_n161), .b(new_n151), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n157), .b(new_n160), .c(new_n147), .d(new_n162), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n160), .b(new_n157), .c(new_n147), .d(new_n162), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n154), .clkout(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  nanp03aa1n02x5               g074(.a(new_n163), .b(new_n166), .c(new_n169), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n169), .b(new_n163), .c(new_n166), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n154), .b(new_n167), .c(new_n168), .d(new_n155), .out0(new_n173));
  nano22aa1n02x4               g078(.a(new_n146), .b(new_n162), .c(new_n173), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n173), .b(new_n160), .c(new_n144), .d(new_n162), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n167), .b(new_n154), .c(new_n168), .o1(new_n177));
  nanp03aa1n02x5               g082(.a(new_n175), .b(new_n176), .c(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n122), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n173), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n131), .clkout(new_n187));
  aoai13aa1n02x5               g092(.a(new_n162), .b(new_n143), .c(new_n145), .d(new_n187), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n177), .b(new_n186), .c(new_n188), .d(new_n159), .o1(new_n189));
  xroi22aa1d04x5               g094(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n185), .d(new_n174), .o1(new_n191));
  oai022aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(new_n180), .c(\b[17] ), .out0(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(new_n196), .clkout(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n191), .c(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n194), .clkout(new_n200));
  aoi012aa1n02x5               g105(.a(new_n196), .b(new_n191), .c(new_n193), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  nano22aa1n02x4               g109(.a(new_n201), .b(new_n200), .c(new_n204), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n182), .b(new_n181), .o1(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n197), .b(new_n207), .c(new_n178), .d(new_n190), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n204), .b(new_n208), .c(new_n200), .o1(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n205), .o1(\s[20] ));
  nano23aa1n02x4               g115(.a(new_n194), .b(new_n202), .c(new_n203), .d(new_n195), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n190), .b(new_n211), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n189), .c(new_n185), .d(new_n174), .o1(new_n214));
  nona23aa1n02x4               g119(.a(new_n203), .b(new_n195), .c(new_n194), .d(new_n202), .out0(new_n215));
  aoi012aa1n02x5               g120(.a(new_n202), .b(new_n194), .c(new_n203), .o1(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n215), .c(new_n193), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n214), .c(new_n218), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n219), .clkout(new_n223));
  aobi12aa1n02x5               g128(.a(new_n221), .b(new_n214), .c(new_n218), .out0(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  nano22aa1n02x4               g130(.a(new_n224), .b(new_n223), .c(new_n225), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n221), .b(new_n217), .c(new_n178), .d(new_n213), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n223), .o1(new_n228));
  norp02aa1n02x5               g133(.a(new_n228), .b(new_n226), .o1(\s[22] ));
  nano22aa1n02x4               g134(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n230));
  and003aa1n02x5               g135(.a(new_n190), .b(new_n230), .c(new_n211), .o(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n189), .c(new_n185), .d(new_n174), .o1(new_n232));
  oao003aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n223), .carry(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n217), .c(new_n230), .o1(new_n235));
  xnrc02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .out0(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n235), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoi012aa1n02x5               g145(.a(new_n236), .b(new_n232), .c(new_n235), .o1(new_n241));
  xnrc02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  nano22aa1n02x4               g147(.a(new_n241), .b(new_n240), .c(new_n242), .out0(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n235), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n237), .b(new_n244), .c(new_n178), .d(new_n231), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n242), .b(new_n245), .c(new_n240), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n243), .o1(\s[24] ));
  norp02aa1n02x5               g152(.a(new_n242), .b(new_n236), .o1(new_n248));
  nano22aa1n02x4               g153(.a(new_n212), .b(new_n230), .c(new_n248), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n189), .c(new_n185), .d(new_n174), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n216), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n230), .b(new_n251), .c(new_n211), .d(new_n207), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n248), .clkout(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n240), .carry(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .d(new_n233), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .out0(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n250), .c(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  aoi012aa1n02x5               g166(.a(new_n257), .b(new_n250), .c(new_n256), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  nano22aa1n02x4               g168(.a(new_n262), .b(new_n261), .c(new_n263), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n258), .b(new_n255), .c(new_n178), .d(new_n249), .o1(new_n265));
  aoi012aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n266), .b(new_n264), .o1(\s[26] ));
  norp02aa1n02x5               g172(.a(new_n263), .b(new_n257), .o1(new_n268));
  nano32aa1n02x4               g173(.a(new_n212), .b(new_n268), .c(new_n230), .d(new_n248), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n189), .c(new_n185), .d(new_n174), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n271));
  aobi12aa1n02x5               g176(.a(new_n271), .b(new_n255), .c(new_n268), .out0(new_n272));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n270), .c(new_n272), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n273), .clkout(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n248), .b(new_n234), .c(new_n217), .d(new_n230), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n268), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n271), .b(new_n280), .c(new_n279), .d(new_n254), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n274), .b(new_n281), .c(new_n178), .d(new_n269), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n278), .b(new_n282), .c(new_n277), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n274), .b(new_n270), .c(new_n272), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n277), .c(new_n278), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g191(.a(new_n278), .b(new_n277), .c(new_n274), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n281), .c(new_n178), .d(new_n269), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n270), .c(new_n272), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n275), .b(new_n290), .c(new_n278), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n281), .c(new_n178), .d(new_n269), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n270), .c(new_n272), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  norb03aa1n02x5               g209(.a(new_n287), .b(new_n299), .c(new_n290), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n270), .c(new_n272), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n307));
  nano22aa1n02x4               g212(.a(new_n306), .b(new_n304), .c(new_n307), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n305), .b(new_n281), .c(new_n178), .d(new_n269), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n304), .b(new_n309), .c(new_n307), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g220(.a(new_n116), .b(new_n117), .c(new_n107), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g222(.a(\a[6] ), .b(\b[5] ), .c(new_n316), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g224(.a(new_n110), .b(new_n318), .c(new_n111), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g226(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


