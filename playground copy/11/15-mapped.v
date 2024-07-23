// Benchmark "adder" written by ABC on Thu Jul 11 11:41:37 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n314, new_n316, new_n317, new_n318, new_n319, new_n321,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n02x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  norp02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n112), .b(new_n115), .c(new_n116), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n113), .b(new_n118), .c(new_n114), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n120), .b(new_n112), .c(new_n119), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n107), .c(new_n117), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nano23aa1n02x4               g033(.a(new_n125), .b(new_n127), .c(new_n128), .d(new_n126), .out0(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n129), .clkout(new_n130));
  aoi012aa1n02x5               g035(.a(new_n125), .b(new_n127), .c(new_n126), .o1(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n122), .c(new_n130), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnrc02aa1n02x5               g044(.a(new_n136), .b(new_n139), .out0(\s[12] ));
  nona23aa1n02x4               g045(.a(new_n138), .b(new_n135), .c(new_n134), .d(new_n137), .out0(new_n141));
  aoi012aa1n02x5               g046(.a(new_n137), .b(new_n134), .c(new_n138), .o1(new_n142));
  oai012aa1n02x5               g047(.a(new_n142), .b(new_n141), .c(new_n131), .o1(new_n143));
  nano23aa1n02x4               g048(.a(new_n134), .b(new_n137), .c(new_n138), .d(new_n135), .out0(new_n144));
  nano22aa1n02x4               g049(.a(new_n122), .b(new_n129), .c(new_n144), .out0(new_n145));
  norp02aa1n02x5               g050(.a(new_n145), .b(new_n143), .o1(new_n146));
  norp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  nanp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n146), .b(new_n149), .c(new_n148), .out0(\s[13] ));
  oaoi13aa1n02x5               g055(.a(new_n147), .b(new_n149), .c(new_n145), .d(new_n143), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoi112aa1n02x5               g057(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n135), .b(new_n134), .out0(new_n154));
  oai112aa1n02x5               g059(.a(new_n154), .b(new_n139), .c(new_n153), .d(new_n125), .o1(new_n155));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n147), .b(new_n156), .c(new_n157), .d(new_n149), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  aoi012aa1n02x5               g064(.a(new_n156), .b(new_n147), .c(new_n157), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n155), .d(new_n142), .o1(new_n161));
  nano32aa1n02x4               g066(.a(new_n122), .b(new_n158), .c(new_n129), .d(new_n144), .out0(new_n162));
  norp02aa1n02x5               g067(.a(new_n162), .b(new_n161), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  oaoi13aa1n02x5               g071(.a(new_n165), .b(new_n166), .c(new_n162), .d(new_n161), .o1(new_n167));
  xnrb03aa1n02x5               g072(.a(new_n167), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nano23aa1n02x4               g075(.a(new_n165), .b(new_n169), .c(new_n170), .d(new_n166), .out0(new_n171));
  nanp02aa1n02x5               g076(.a(new_n171), .b(new_n158), .o1(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n129), .c(new_n144), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n121), .c(new_n107), .d(new_n117), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n160), .clkout(new_n175));
  aoai13aa1n02x5               g080(.a(new_n171), .b(new_n175), .c(new_n143), .d(new_n158), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n169), .b(new_n165), .c(new_n170), .o1(new_n177));
  nanp03aa1n02x5               g082(.a(new_n174), .b(new_n176), .c(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  aobi12aa1n02x5               g089(.a(new_n177), .b(new_n161), .c(new_n171), .out0(new_n185));
  xroi22aa1d04x5               g090(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n186), .clkout(new_n187));
  norp02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n189));
  norp02aa1n02x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n187), .c(new_n185), .d(new_n174), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoi112aa1n02x5               g103(.a(new_n194), .b(new_n198), .c(new_n191), .d(new_n195), .o1(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n194), .clkout(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n190), .clkout(new_n201));
  norb02aa1n02x5               g106(.a(new_n195), .b(new_n194), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n201), .c(new_n178), .d(new_n186), .o1(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n198), .clkout(new_n204));
  aoi012aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n200), .o1(new_n205));
  norp02aa1n02x5               g110(.a(new_n205), .b(new_n199), .o1(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n194), .b(new_n196), .c(new_n197), .d(new_n195), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n186), .b(new_n207), .o1(new_n208));
  oai112aa1n02x5               g113(.a(new_n202), .b(new_n198), .c(new_n189), .d(new_n188), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n196), .b(new_n194), .c(new_n197), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n209), .b(new_n210), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n208), .c(new_n185), .d(new_n174), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xnrc02aa1n02x5               g121(.a(\b[21] ), .b(\a[22] ), .out0(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoi112aa1n02x5               g123(.a(new_n215), .b(new_n218), .c(new_n213), .d(new_n216), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n215), .clkout(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n208), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n216), .b(new_n211), .c(new_n178), .d(new_n221), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n217), .b(new_n222), .c(new_n220), .o1(new_n223));
  norp02aa1n02x5               g128(.a(new_n223), .b(new_n219), .o1(\s[22] ));
  nano32aa1n02x4               g129(.a(new_n187), .b(new_n218), .c(new_n207), .d(new_n216), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  nanb02aa1n02x5               g131(.a(new_n217), .b(new_n216), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .carry(new_n229));
  aobi12aa1n02x5               g134(.a(new_n229), .b(new_n228), .c(new_n211), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n226), .c(new_n185), .d(new_n174), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n233), .clkout(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n230), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n234), .b(new_n238), .c(new_n178), .d(new_n225), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n235), .clkout(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n239), .c(new_n237), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n241), .b(new_n236), .o1(\s[24] ));
  nanp02aa1n02x5               g147(.a(new_n235), .b(new_n234), .o1(new_n243));
  norp03aa1n02x5               g148(.a(new_n208), .b(new_n227), .c(new_n243), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  aoi112aa1n02x5               g150(.a(new_n227), .b(new_n243), .c(new_n209), .d(new_n210), .o1(new_n246));
  oao003aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .c(new_n237), .carry(new_n247));
  oai012aa1n02x5               g152(.a(new_n247), .b(new_n243), .c(new_n229), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n246), .b(new_n248), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n185), .d(new_n174), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n252), .clkout(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n249), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n253), .b(new_n257), .c(new_n178), .d(new_n244), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n254), .clkout(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n260), .b(new_n255), .o1(\s[26] ));
  and002aa1n02x5               g166(.a(new_n254), .b(new_n253), .o(new_n262));
  nona23aa1n02x4               g167(.a(new_n221), .b(new_n262), .c(new_n243), .d(new_n227), .out0(new_n263));
  oao003aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  oaoi13aa1n02x5               g170(.a(new_n265), .b(new_n262), .c(new_n246), .d(new_n248), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n263), .c(new_n185), .d(new_n174), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n269), .clkout(new_n270));
  xorc02aa1n02x5               g175(.a(\a[28] ), .b(\b[27] ), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n263), .clkout(new_n273));
  oai012aa1n02x5               g178(.a(new_n262), .b(new_n246), .c(new_n248), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n274), .b(new_n264), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n178), .d(new_n273), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n272), .b(new_n277), .c(new_n270), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n276), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n278), .b(new_n279), .o1(\s[28] ));
  nano22aa1n02x4               g185(.a(new_n272), .b(new_n270), .c(new_n276), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n281), .b(new_n275), .c(new_n178), .d(new_n273), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n270), .carry(new_n283));
  xorc02aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .out0(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n284), .clkout(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n282), .c(new_n283), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n283), .clkout(new_n287));
  aoi112aa1n02x5               g192(.a(new_n284), .b(new_n287), .c(new_n267), .d(new_n281), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g195(.a(new_n285), .b(new_n271), .c(new_n276), .d(new_n270), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n275), .c(new_n178), .d(new_n273), .o1(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  xorc02aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .out0(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n295), .clkout(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n292), .c(new_n294), .o1(new_n297));
  aoi112aa1n02x5               g202(.a(new_n295), .b(new_n293), .c(new_n267), .d(new_n291), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n297), .b(new_n298), .o1(\s[30] ));
  and003aa1n02x5               g204(.a(new_n281), .b(new_n295), .c(new_n284), .o(new_n300));
  oaoi03aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  160nm_ficinv00aa1n08x5       g207(.clk(new_n302), .clkout(new_n303));
  aoi112aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n267), .d(new_n300), .o1(new_n304));
  aoai13aa1n02x5               g209(.a(new_n300), .b(new_n275), .c(new_n178), .d(new_n273), .o1(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n301), .clkout(new_n306));
  aoi012aa1n02x5               g211(.a(new_n302), .b(new_n305), .c(new_n306), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g217(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(new_n313));
  oai012aa1n02x5               g218(.a(new_n313), .b(\b[4] ), .c(\a[5] ), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g220(.a(new_n110), .b(new_n111), .out0(new_n316));
  160nm_ficinv00aa1n08x5       g221(.clk(new_n316), .clkout(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n113), .c(new_n314), .d(new_n114), .o1(new_n318));
  aoi112aa1n02x5               g223(.a(new_n317), .b(new_n113), .c(new_n314), .d(new_n114), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[7] ));
  nanb02aa1n02x5               g225(.a(new_n108), .b(new_n109), .out0(new_n321));
  160nm_ficinv00aa1n08x5       g226(.clk(new_n110), .clkout(new_n322));
  xobna2aa1n03x5               g227(.a(new_n321), .b(new_n318), .c(new_n322), .out0(\s[8] ));
  xnrb03aa1n02x5               g228(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


