// Benchmark "adder" written by ABC on Thu Jul 11 11:17:18 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n307, new_n308, new_n310;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n02x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  norb02aa1n02x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  nano22aa1n02x4               g013(.a(new_n108), .b(new_n100), .c(new_n103), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  aoi012aa1n02x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  oai012aa1n02x5               g023(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(new_n118), .c(new_n113), .o1(new_n120));
  aoi112aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n121));
  aoi112aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  oai112aa1n02x5               g027(.a(new_n103), .b(new_n100), .c(new_n122), .d(new_n104), .o1(new_n123));
  nona22aa1n02x4               g028(.a(new_n123), .b(new_n121), .c(new_n98), .out0(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n124), .b(new_n120), .c(new_n109), .o(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnrc02aa1n02x5               g035(.a(new_n127), .b(new_n130), .out0(\s[10] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoi012aa1n02x5               g039(.a(new_n128), .b(new_n97), .c(new_n129), .o1(new_n135));
  nano23aa1n02x4               g040(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n126), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n134), .b(new_n137), .c(new_n135), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n132), .clkout(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n134), .clkout(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n140), .c(new_n137), .d(new_n135), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n143), .out0(new_n147));
  oai112aa1n02x5               g052(.a(new_n147), .b(new_n134), .c(new_n145), .d(new_n128), .o1(new_n148));
  nona22aa1n02x4               g053(.a(new_n148), .b(new_n144), .c(new_n143), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  and003aa1n02x5               g055(.a(new_n136), .b(new_n134), .c(new_n147), .o(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n152), .b(new_n150), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n159), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n152), .d(new_n150), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n174));
  nano23aa1n02x4               g079(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  nano32aa1n02x4               g081(.a(new_n176), .b(new_n136), .c(new_n147), .d(new_n134), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n165), .b(new_n166), .out0(new_n179));
  norp03aa1n02x5               g084(.a(new_n162), .b(new_n169), .c(new_n179), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n181));
  obai22aa1n02x7               g086(.a(new_n175), .b(new_n161), .c(\a[16] ), .d(\b[15] ), .out0(new_n182));
  aoi112aa1n02x5               g087(.a(new_n182), .b(new_n181), .c(new_n149), .d(new_n180), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n183), .c(new_n178), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  nanp02aa1n02x5               g091(.a(new_n183), .b(new_n178), .o1(new_n187));
  norp02aa1n02x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[17] ), .clkout(new_n191));
  xroi22aa1d04x5               g096(.a(new_n191), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[17] ), .clkout(new_n194));
  oao003aa1n02x5               g099(.a(new_n186), .b(new_n194), .c(new_n188), .carry(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n195), .clkout(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n178), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n200), .b(new_n204), .c(new_n197), .d(new_n201), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n204), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n192), .b(new_n208), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n210));
  norp02aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n201), .b(new_n200), .out0(new_n213));
  oai112aa1n02x5               g118(.a(new_n213), .b(new_n204), .c(new_n212), .d(new_n211), .o1(new_n214));
  nona22aa1n02x4               g119(.a(new_n214), .b(new_n210), .c(new_n202), .out0(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n209), .c(new_n183), .d(new_n178), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norp02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n219), .b(new_n223), .c(new_n217), .d(new_n220), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n223), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  nona23aa1n02x4               g131(.a(new_n222), .b(new_n220), .c(new_n219), .d(new_n221), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  nanp03aa1n02x5               g133(.a(new_n192), .b(new_n228), .c(new_n208), .o1(new_n229));
  160nm_fiao0012aa1n02p5x5     g134(.a(new_n221), .b(new_n219), .c(new_n222), .o(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n215), .c(new_n228), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n229), .c(new_n183), .d(new_n178), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(\s[24] ));
  and002aa1n02x5               g144(.a(new_n236), .b(new_n235), .o(new_n240));
  nanb03aa1n02x5               g145(.a(new_n209), .b(new_n240), .c(new_n228), .out0(new_n241));
  norp02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n243));
  nanp03aa1n02x5               g148(.a(new_n230), .b(new_n235), .c(new_n236), .o1(new_n244));
  nona22aa1n02x4               g149(.a(new_n244), .b(new_n243), .c(new_n242), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n227), .b(new_n235), .c(new_n236), .out0(new_n246));
  aoi012aa1n02x5               g151(.a(new_n245), .b(new_n215), .c(new_n246), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n241), .c(new_n183), .d(new_n178), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  aoi112aa1n02x5               g157(.a(new_n250), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(\s[26] ));
  and002aa1n02x5               g160(.a(new_n252), .b(new_n251), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n229), .b(new_n256), .c(new_n240), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[25] ), .b(\a[26] ), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n256), .b(new_n245), .c(new_n215), .d(new_n246), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n259), .c(new_n258), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n187), .d(new_n257), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n261), .b(new_n262), .c(new_n187), .d(new_n257), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n263), .b(new_n264), .out0(\s[27] ));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[28] ), .b(\b[27] ), .out0(new_n267));
  nona22aa1n02x4               g172(.a(new_n263), .b(new_n267), .c(new_n266), .out0(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n266), .clkout(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n267), .clkout(new_n270));
  aoi012aa1n02x5               g175(.a(new_n270), .b(new_n263), .c(new_n269), .o1(new_n271));
  norb02aa1n02x5               g176(.a(new_n268), .b(new_n271), .out0(\s[28] ));
  and002aa1n02x5               g177(.a(new_n267), .b(new_n262), .o(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n261), .c(new_n187), .d(new_n257), .o1(new_n274));
  oaoi03aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  xorc02aa1n02x5               g181(.a(\a[29] ), .b(\b[28] ), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aoi012aa1n02x5               g183(.a(new_n278), .b(new_n274), .c(new_n276), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n274), .b(new_n275), .c(new_n277), .out0(new_n280));
  norb02aa1n02x5               g185(.a(new_n280), .b(new_n279), .out0(\s[29] ));
  xorb03aa1n02x5               g186(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g187(.a(new_n278), .b(new_n262), .c(new_n267), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n283), .b(new_n261), .c(new_n187), .d(new_n257), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n285));
  xorc02aa1n02x5               g190(.a(\a[30] ), .b(\b[29] ), .out0(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n284), .c(new_n285), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n285), .clkout(new_n289));
  nona22aa1n02x4               g194(.a(new_n284), .b(new_n289), .c(new_n286), .out0(new_n290));
  norb02aa1n02x5               g195(.a(new_n290), .b(new_n288), .out0(\s[30] ));
  xnrc02aa1n02x5               g196(.a(\b[30] ), .b(\a[31] ), .out0(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n292), .clkout(new_n293));
  nano32aa1n02x4               g198(.a(new_n287), .b(new_n277), .c(new_n267), .d(new_n262), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n261), .c(new_n187), .d(new_n257), .o1(new_n295));
  oaoi03aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .o1(new_n296));
  nona22aa1n02x4               g201(.a(new_n295), .b(new_n296), .c(new_n293), .out0(new_n297));
  160nm_ficinv00aa1n08x5       g202(.clk(new_n296), .clkout(new_n298));
  aoi012aa1n02x5               g203(.a(new_n292), .b(new_n295), .c(new_n298), .o1(new_n299));
  norb02aa1n02x5               g204(.a(new_n297), .b(new_n299), .out0(\s[31] ));
  xnrb03aa1n02x5               g205(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g209(.a(new_n106), .b(new_n120), .c(new_n107), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g211(.a(new_n108), .b(new_n120), .out0(new_n307));
  nona22aa1n02x4               g212(.a(new_n307), .b(new_n122), .c(new_n104), .out0(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n101), .b(new_n308), .c(new_n102), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(new_n310), .b(new_n100), .out0(\s[8] ));
  xorb03aa1n02x5               g216(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


